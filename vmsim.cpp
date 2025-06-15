#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <cstdint>
#include <algorithm>

using namespace std;

// 페이지 크기와 관련 상수
const int PAGE_OFFSET_BITS = 12;
const int PAGE_SIZE = 1 << PAGE_OFFSET_BITS; // 4KB
const int LEVEL1_BITS = 10;
const int LEVEL2_BITS = 10;
const uint32_t LEVEL1_MASK = (1 << LEVEL1_BITS) - 1;
const uint32_t LEVEL2_MASK = (1 << LEVEL2_BITS) - 1;
const uint32_t OFFSET_MASK = (1 << PAGE_OFFSET_BITS) - 1;

// TLB 엔트리
struct TLBEntry {
    uint32_t vpn;  // Virtual Page Number
    uint32_t ppn;  // Physical Page Number
    bool valid;
    TLBEntry() : vpn(0), ppn(0), valid(false) {}
};

// 페이지 테이블 엔트리
struct PageTableEntry {
    uint32_t ppn;
    bool valid;
    PageTableEntry() : ppn(0), valid(false) {}
};

// S3-FIFO 구조체
template<typename T>
class S3FIFO {
public:
    size_t total_capacity;
    size_t small_size;
    size_t main_size;
    size_t ghost_size;
    
    deque<T> small_fifo;
    deque<T> main_fifo;
    unordered_set<T> ghost_fifo;
    unordered_map<T, int> freq; // 접근 빈도
    
    S3FIFO(size_t capacity) : total_capacity(capacity) {
        small_size = max(1UL, capacity / 10); // Small FIFO는 전체의 10%
        main_size = max(1UL, total_capacity - small_size); 
        ghost_size = max(1UL, total_capacity - small_size);
    }
    
    bool contains(const T& item) {
        return find(small_fifo.begin(), small_fifo.end(), item) != small_fifo.end() ||
               find(main_fifo.begin(), main_fifo.end(), item) != main_fifo.end();
    }
    
    void access(const T& item) {
        freq[item]++;
    }
    
    pair<bool, T> insert(const T& item) {
        bool evicted = false;
        T evicted_item = T();
        
        // 이미 캐시에 있으면 접근 빈도만 증가
        if (contains(item)) {
            access(item);
            return {false, evicted_item};
        }

        if(ghost_fifo.count(item)) {
            ghost_fifo.erase(item);
            main_fifo.push_back(item);
            freq[item]=0;
            return {false, evicted_item};
        }
        
        // Small FIFO가 가득 찼으면 eviction
        if (small_fifo.size() >= small_size) {
            T victim = small_fifo.front();
            small_fifo.pop_front();
            
            if(freq[victim] >=1){
                main_fifo.push_back(victim);
                freq[victim]=0;
            }else {
                if(ghost_fifo.size() >= ghost_size) {
                    ghost_fifo.erase(ghost_fifo.begin());
                }
                ghost_fifo.insert(victim);
                evicted = true;
                evicted_item = victim;
            }
        }
        
        // 새 항목은 small FIFO에 추가
        small_fifo.push_back(item);
        freq[item] = 0;
        
        return {evicted, evicted_item};
    }
    
    void clear() {
        small_fifo.clear();
        main_fifo.clear();
        ghost_fifo.clear();
        freq.clear();
    }
};

// 가상 메모리 시뮬레이터
class VMSimulator {
private:
    // Two-level 페이지 테이블
    vector<vector<PageTableEntry>> page_table;
    
    // TLB
    S3FIFO<uint32_t> tlb_s3fifo;
    unordered_map<uint32_t, uint32_t> tlb_map; // VPN -> PPN
    
    // Physical frames
    vector<uint32_t> frames; // frame[i] = VPN using frame i
    S3FIFO<uint32_t> frame_s3fifo; // VPN 기반 S3-FIFO
    unordered_map<uint32_t, uint32_t> vpn_to_frame; // VPN -> frame number

    uint32_t findFreeFrame(){
        for(int i=0; i<frames.size(); i++){
            if(frames[i] == UINT32_MAX){
                return i;
            }
        }
        return UINT32_MAX;
    }
    void freeFrame(uint32_t frame_num){
        frames[frame_num] = UINT32_MAX;
    }
    void occupyFrame(uint32_t frame_num, uint32_t vpn){
        frames[frame_num] = vpn;
    }
    
    // 통계
    int total_references;
    int tlb_hits;
    int tlb_misses;
    int page_faults;
    
    // 설정
    int num_frames;
    int tlb_size;
    int allocated_frames;
    
public:
    VMSimulator(int frames_count, int tlb_sz) 
        : page_table(1 << LEVEL1_BITS, vector<PageTableEntry>(1 << LEVEL2_BITS)),
          tlb_s3fifo(tlb_sz),
          frames(frames_count, UINT32_MAX),
          frame_s3fifo(frames_count),
          total_references(0),
          tlb_hits(0),
          tlb_misses(0),
          page_faults(0),
          num_frames(frames_count),
          tlb_size(tlb_sz),
          allocated_frames(0) {}
    
    void access_memory(uint32_t vaddr) {
        total_references++;
        
        // 주소 분해
        uint32_t vpn = vaddr >> PAGE_OFFSET_BITS;
        uint32_t offset = vaddr & OFFSET_MASK;
        uint32_t level1_idx = (vpn >> LEVEL2_BITS) & LEVEL1_MASK;
        uint32_t level2_idx = vpn & LEVEL2_MASK;
        
        uint32_t frame_num = 0;  // 초기화 추가
        bool tlb_hit = false;
        bool page_fault = false;
        uint32_t evicted_vpn = UINT32_MAX;
        bool frame_evicted = false;
        
        // TLB 확인
        if (tlb_map.count(vpn) > 0) {
            tlb_hit = true;
            tlb_hits++;
            frame_num = tlb_map[vpn];
            tlb_s3fifo.access(vpn);
            frame_s3fifo.access(vpn);
        } else {
            tlb_misses++;
            
            // 페이지 테이블 확인
            if (page_table[level1_idx][level2_idx].valid) {
                frame_num = page_table[level1_idx][level2_idx].ppn;
            } else {
                // Page fault 처리
                page_fault = true;
                page_faults++;
        
                if(frame_s3fifo.small_fifo.size() >= frame_s3fifo.small_size){
                    uint32_t victim = frame_s3fifo.small_fifo.front();
                    frame_s3fifo.small_fifo.pop_front();
                    if(frame_s3fifo.freq[victim] >=1){
                        frame_s3fifo.main_fifo.push_back(victim);
                        frame_s3fifo.freq[victim]=0;
                    }else {
                        if(frame_s3fifo.ghost_fifo.size() >= frame_s3fifo.ghost_size) {
                            frame_s3fifo.ghost_fifo.erase(frame_s3fifo.ghost_fifo.begin());
                        }
                        frame_s3fifo.ghost_fifo.insert(victim);
                        frame_evicted = true;
                        evicted_vpn = vpn_to_frame[victim];
                        freeFrame(vpn_to_frame[victim]);
                        vpn_to_frame.erase(victim);
                    }
                }
                
                // 프레임이 모두 사용 중인지 확인
                if (allocated_frames < num_frames && !frame_evicted) {
                    // 새 프레임 할당
                    frame_num =findFreeFrame();
                    occupyFrame(frame_num, frame_num);
                    frame_s3fifo.insert(vpn);
                } else {
                    // 프레임 교체 필요

                        // evicted VPN이 사용하던 프레임 찾기
                        frame_num = findFreeFrame();
                        
                        // 이전 매핑 제거
                        uint32_t ev_l1 = (evicted_vpn >> LEVEL2_BITS) & LEVEL1_MASK;
                        uint32_t ev_l2 = evicted_vpn & LEVEL2_MASK;
                        page_table[ev_l1][ev_l2].valid = false;
                        
                        // TLB에서도 제거
                        tlb_map.erase(evicted_vpn);
                        vpn_to_frame.erase(evicted_vpn);
                        
                        // 프레임에 새 VPN 할당
                        occupyFrame(frame_num, frame_num);
                    
                }
                
                // 새 매핑 설정
                page_table[level1_idx][level2_idx].ppn = frame_num;
                page_table[level1_idx][level2_idx].valid = true;
                vpn_to_frame[vpn] = frame_num;
            }
            
            // TLB에 추가
            auto [tlb_evicted, evicted_tlb_vpn] = tlb_s3fifo.insert(vpn);
            if (tlb_evicted) {
                tlb_map.erase(evicted_tlb_vpn);
            }
            tlb_map[vpn] = frame_num;
        }
        
        // 물리 주소 계산 - 프레임 번호를 4KB 단위로 이동
        uint32_t paddr = (frame_num << PAGE_OFFSET_BITS) | offset;
        
        // 출력 - 대문자 유지
        cout << "0x" << setfill('0') << setw(8) << hex << uppercase << vaddr 
             << " -> 0x" << setfill('0') << setw(8) << hex << uppercase << paddr
             << ", TLB " << (tlb_hit ? "hit" : "miss")
             << ", " << (page_fault ? "Page fault" : "No page fault");
        
        if (frame_evicted && evicted_vpn != UINT32_MAX) {
            cout << ", Evicted 0x" << setfill('0') << setw(8) << hex << uppercase 
                 << (evicted_vpn << PAGE_OFFSET_BITS);
        }
        
        cout << endl;
    }
    
    void print_statistics() {
        cout << "Total references: " << dec << total_references << endl;
        cout << "TLB hits: " << tlb_hits << endl;
        cout << "TLB misses: " << tlb_misses << endl;
        cout << "TLB hit ratio: " << fixed << setprecision(1) 
             << (tlb_hits * 100.0 / total_references) << "%" << endl;
        cout << "Page faults: " << page_faults << endl;
        cout << "Page fault rate: " << fixed << setprecision(1) 
             << (page_faults * 100.0 / total_references) << "%" << endl;
    }
};

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <num_frames> <tlb_size> <algorithm>" << endl;
        return 1;
    }
    
    int num_frames = stoi(argv[1]);
    int tlb_size = stoi(argv[2]);
    string algorithm = argv[3];
    
    if (algorithm != "S3FIFO") {
        cerr << "Only S3FIFO algorithm is supported" << endl;
        return 1;
    }
    
    VMSimulator simulator(num_frames, tlb_size);
    
    string line;
    while (getline(cin, line)) {
        uint32_t vaddr = stoul(line, nullptr, 16);
        simulator.access_memory(vaddr);
    }
    
    simulator.print_statistics();
    
    return 0;
}
