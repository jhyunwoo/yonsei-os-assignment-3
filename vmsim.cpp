#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <cstdint>
#include <algorithm>
#include <memory>

using namespace std;

constexpr int  PAGE_OFFSET_BITS = 12;
constexpr int  LEVEL1_BITS = 10;
constexpr int  LEVEL2_BITS = 10;
constexpr uint32_t LEVEL1_MASK = (1u << LEVEL1_BITS) - 1;
constexpr uint32_t LEVEL2_MASK = (1u << LEVEL2_BITS) - 1;
constexpr uint32_t OFFSET_MASK = (1u << PAGE_OFFSET_BITS) - 1;

struct PageTableEntry { uint32_t ppn = 0; bool valid = false; };

// S3-FIFO 캐시 구현 클래스
template<typename T>
class S3FIFO {
public:
    size_t total_capacity;    // 전체 캐시 용량
    size_t small_size;        // Small FIFO 크기
    size_t main_size;         // Main FIFO 크기
    size_t ghost_size;        // Ghost FIFO 크기
    
    deque<T> small_fifo;      // Small FIFO (최근에 들어온 항목들)
    deque<T> main_fifo;       // Main FIFO (자주 접근되는 항목들)
    unordered_set<T> ghost_fifo;  // Ghost FIFO (방출된 항목들의 기록)
    unordered_map<T, int> freq;   // 각 항목의 접근 빈도
    
    // 생성자: 전체 용량을 받아 각 FIFO의 크기를 초기화
    S3FIFO(size_t capacity) : total_capacity(capacity) {
        small_size = max(1UL, capacity / 10);  // Small FIFO는 전체의 10%
        main_size = max(1UL, total_capacity - small_size); 
        ghost_size = max(1UL, total_capacity - small_size);
    }
    
    // 항목이 캐시에 있는지 확인
    bool contains(const T& item) {
        return find(small_fifo.begin(), small_fifo.end(), item) != small_fifo.end() ||
               find(main_fifo.begin(), main_fifo.end(), item) != main_fifo.end();
    }
    
    // 항목 접근 시 빈도 증가
    void access(const T& item) {
        freq[item]++;
    }
    
    // 새 항목 삽입 및 필요시 방출
    pair<bool, T> insert(const T& item) {
        bool evicted = false;
        T evicted_item = T();
        
        // 이미 캐시에 있으면 접근 빈도만 증가
        if (contains(item)) {
            access(item);
            return {false, evicted_item};
        }

        // Ghost FIFO에 있으면 Main FIFO로 이동
        if(ghost_fifo.count(item)) {
            ghost_fifo.erase(item);
            main_fifo.push_back(item);
            freq[item]=0;
            return {false, evicted_item};
        }
        
        // Small FIFO가 가득 찼으면 방출 처리
        if (small_fifo.size() >= small_size) {
            T victim = small_fifo.front();
            small_fifo.pop_front();
            
            if(freq[victim] >=1){
                // 접근 빈도가 높으면 Main FIFO로 이동
                main_fifo.push_back(victim);
                freq[victim]=0;
            }else {
                // 접근 빈도가 낮으면 Ghost FIFO로 이동
                if(ghost_fifo.size() >= ghost_size) {
                    ghost_fifo.erase(ghost_fifo.begin());
                }
                ghost_fifo.insert(victim);
                evicted = true;
                evicted_item = victim;
            }
        }
        
        // 새 항목은 Small FIFO에 추가
        small_fifo.push_back(item);
        freq[item] = 0;
        
        return {evicted, evicted_item};
    }
    
    // 캐시 초기화
    void clear() {
        small_fifo.clear();
        main_fifo.clear();
        ghost_fifo.clear();
        freq.clear();
    }
};

// 가상 메모리 시뮬레이터 클래스
class VMSimulator {
private:
    // Two-level 페이지 테이블
    vector<vector<PageTableEntry>> page_table;
    
    // TLB (Translation Lookaside Buffer)
    S3FIFO<uint32_t> tlb_s3fifo;  // TLB 캐시
    unordered_map<uint32_t, uint32_t> tlb_map;  // VPN -> PPN 매핑
    
    // 물리 메모리 프레임 관리
    vector<uint32_t> frames;  // frame[i] = i번 프레임을 사용 중인 VPN
    S3FIFO<uint32_t> frame_s3fifo;  // 프레임 교체를 위한 S3-FIFO
    unordered_map<uint32_t, uint32_t> vpn_to_frame;  // VPN -> 프레임 번호 매핑

    // 사용 가능한 프레임 찾기
    uint32_t findFreeFrame(){
        for(size_t i=0; i<frames.size(); i++){
            if(frames[i] == UINT32_MAX){
                return i;
            }
        }
        return UINT32_MAX;
    }

    // 프레임 해제
    void freeFrame(uint32_t frame_num){
        if (frames[frame_num] != UINT32_MAX) {
            --allocated_frames;
            frames[frame_num] = UINT32_MAX;
        }
    }

    // 프레임 할당
    void occupyFrame(uint32_t frame_num, uint32_t vpn){
        if (frames[int(frame_num)] == UINT32_MAX) ++allocated_frames;
        frames[int(frame_num)] = vpn;
    }
    
    // 통계 정보
    int total_references;  // 전체 메모리 참조 횟수
    int tlb_hits;         // TLB 히트 횟수
    int tlb_misses;       // TLB 미스 횟수
    int page_faults;      // 페이지 폴트 횟수
    
    // 설정값
    int num_frames;       // 전체 프레임 수
    int allocated_frames; // 현재 할당된 프레임 수
    
public:
    // 생성자: 프레임 수와 TLB 크기로 초기화
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
          allocated_frames(0) {}
    
    // 메모리 접근 처리
    void access_memory(uint32_t vaddr) {
        total_references++;
        
        // 가상 주소 분해
        uint32_t vpn = vaddr >> PAGE_OFFSET_BITS;  // 가상 페이지 번호
        uint32_t offset = vaddr & OFFSET_MASK;     // 페이지 내 오프셋
        uint32_t level1_idx = (vpn >> LEVEL2_BITS) & LEVEL1_MASK;  // 1단계 인덱스
        uint32_t level2_idx = vpn & LEVEL2_MASK;   // 2단계 인덱스
        
        uint32_t frame_num = 0;  // 물리 프레임 번호
        bool tlb_hit = false;    // TLB 히트 여부
        bool page_fault = false; // 페이지 폴트 여부
        uint32_t evicted_vpn = UINT32_MAX;  // 방출된 VPN
        bool frame_evicted = false;  // 프레임 방출 여부
        
        // TLB 검색
        if (tlb_map.count(vpn) > 0) {
            tlb_hit = true;
            tlb_hits++;
            frame_num = tlb_map[vpn];
            tlb_s3fifo.access(vpn);
            frame_s3fifo.access(vpn);
        } else {
            tlb_misses++;
            // 페이지 테이블 검색
            if (page_table[level1_idx][level2_idx].valid) {
                frame_num = page_table[level1_idx][level2_idx].ppn;
            } else {
                // 페이지 폴트 처리
                page_fault = true;
                page_faults++;

                // Ghost FIFO에 있는 경우 처리
                if(frame_s3fifo.ghost_fifo.count(vpn)){
                    frame_s3fifo.ghost_fifo.erase(vpn);
                    // Main Queue가 가득 찼을 경우 비워준다
                    if(frame_s3fifo.main_fifo.size() >= frame_s3fifo.main_size){
                        uint32_t victim = frame_s3fifo.main_fifo.front();
                        frame_s3fifo.main_fifo.pop_front();
                        frame_s3fifo.ghost_fifo.insert(victim);
                        frame_s3fifo.freq[victim]=0;
                        freeFrame(vpn_to_frame[victim]);
                        vpn_to_frame.erase(victim);
                        frame_evicted = true;
                        evicted_vpn = vpn_to_frame[victim];
                    }
                    // Main Queue에 추가
                    frame_s3fifo.main_fifo.push_back(vpn);
                    frame_s3fifo.freq[vpn]=0;
                    // 페이지 테이블 업데이트
                    vpn_to_frame[vpn] = frame_num;
                    page_table[level1_idx][level2_idx].ppn = frame_num;
                    page_table[level1_idx][level2_idx].valid = true;
                    occupyFrame(frame_num, vpn);
                    
                    // 프레임 할당
                } else if(frame_s3fifo.small_fifo.size() >= frame_s3fifo.small_size){
                    // TLB Small FIFO가 가득 찼을 경우 비워준다
                    if(tlb_s3fifo.small_fifo.size() >= tlb_s3fifo.small_size){
                        uint32_t victim = tlb_s3fifo.small_fifo.front();
                        tlb_s3fifo.small_fifo.pop_front();
                        // 접근 빈도 1이상이면 Main Queue에 추가
                        if(tlb_s3fifo.freq[victim] >=1){
                            // Main Queue가 가득 찼을 경우 비워준다
                            if(tlb_s3fifo.main_fifo.size() >= tlb_s3fifo.main_size){
                                uint32_t victim = tlb_s3fifo.main_fifo.front();
                                tlb_s3fifo.main_fifo.pop_front();
                                tlb_map.erase(victim);
                            }
                            tlb_s3fifo.main_fifo.push_back(victim);
                            tlb_s3fifo.freq[victim]=0;
                        }else {
                            // Ghost Queue에 추가
                            // Ghost Queue가 가득 찼을 경우 비워준다
                            if(tlb_s3fifo.ghost_fifo.size() >= tlb_s3fifo.ghost_size) {
                                tlb_s3fifo.ghost_fifo.erase(tlb_s3fifo.ghost_fifo.begin());
                            }
                            tlb_s3fifo.ghost_fifo.insert(victim);
                            tlb_s3fifo.freq[victim]=0;
                            tlb_map.erase(victim);
                        }
                    }
                    tlb_s3fifo.small_fifo.push_back(vpn); // Small Queue에 추가
                    tlb_s3fifo.freq[vpn]=0;

                    // 프레임 Small FIFO 처리
                    uint32_t victim = frame_s3fifo.small_fifo.front();
                    frame_s3fifo.small_fifo.pop_front();
                    // 접근 빈도 1이상이면 Main Queue에 추가
                    if(frame_s3fifo.freq[victim] >=1){
                        // Main Queue가 가득 찼을 경우 비워준다
                        if(frame_s3fifo.main_fifo.size() >= frame_s3fifo.main_size){
                            uint32_t victim = frame_s3fifo.main_fifo.front();
                            frame_s3fifo.main_fifo.pop_front();
                            // 프레임 해제
                            freeFrame(vpn_to_frame[victim]);
                            vpn_to_frame.erase(victim);
                            frame_evicted = true;
                            evicted_vpn = vpn_to_frame[victim];
                        }
                        // Main Queue에 추가
                        frame_s3fifo.main_fifo.push_back(victim);
                        frame_s3fifo.freq[victim]=0;
                    }else {
                        // Ghost Queue에 추가
                        // Ghost Queue가 가득 찼을 경우 비워준다
                        if(frame_s3fifo.ghost_fifo.size() >= frame_s3fifo.ghost_size) {
                            frame_s3fifo.ghost_fifo.erase(frame_s3fifo.ghost_fifo.begin());
                        }
                        frame_s3fifo.ghost_fifo.insert(victim);
                        frame_evicted = true;
                        evicted_vpn = vpn_to_frame[victim];
                        // 프레임 해제
                        freeFrame(evicted_vpn);
                        vpn_to_frame.erase(victim);
                    }
                }

                // 새 프레임 할당
                if (allocated_frames < num_frames && !frame_evicted) {
                    frame_num = findFreeFrame();
                    // 에러 처리
                    if(frame_num == UINT32_MAX){
                        cout << "Frame num is UINT32_MAX" << endl;
                        cout << "Free frames: ";
                        for(auto &item : frames) {
                            cout << item << " ";
                        }
                        cout << endl;
                        cout << "VPN to frame: ";
                        for(auto &item : vpn_to_frame) {
                            cout << item.first << " -> " << item.second << " ";
                        }
                        cout << endl;
                    }
                    // 프레임 할당
                    occupyFrame(frame_num, vpn);
                    frame_s3fifo.insert(vpn);
                } else {
                    // 프레임 교체
                    frame_num = findFreeFrame();
                    tlb_map.erase(evicted_vpn);
                    vpn_to_frame.erase(evicted_vpn);
                    // 프레임 할당
                    occupyFrame(frame_num, vpn);
                }
                
                // 페이지 테이블 업데이트
                page_table[level1_idx][level2_idx].ppn = frame_num;
                page_table[level1_idx][level2_idx].valid = true;
                vpn_to_frame[vpn] = frame_num;
            }
            
            // TLB 업데이트
            auto [tlb_evicted, evicted_tlb_vpn] = tlb_s3fifo.insert(vpn);
            if (tlb_evicted) {
                // 프레임 해제
                tlb_map.erase(evicted_tlb_vpn);
                freeFrame(vpn_to_frame[evicted_tlb_vpn]);
                vpn_to_frame.erase(evicted_tlb_vpn);
                uint32_t v1 = (evicted_tlb_vpn >> LEVEL2_BITS) & LEVEL1_MASK;
                uint32_t v2 =  evicted_tlb_vpn & LEVEL2_MASK;
                page_table[v1][v2].valid = false; 
            }
            tlb_map[vpn] = frame_num;
        }
        
        // 물리 주소 계산
        uint32_t paddr = (frame_num << PAGE_OFFSET_BITS) | offset;
        
        // 결과 출력
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
    
    // 통계 정보 출력
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

// 32비트 정수를 16진수로 출력하는 헬퍼 함수
static inline void print_hex32(uint32_t v)
{
    cout << "0x" << uppercase << hex << setw(8) << setfill('0') << v;
}

// 페이지 교체 정책 구현
namespace policy {

// FIFO (First In First Out) 정책 구현
struct FIFO {
    explicit FIFO(size_t c) : cap(c) {}  // 생성자
    bool access(uint32_t k) { return idx.count(k); }  // 항목 존재 여부 확인
    pair<bool, uint32_t> insert(uint32_t k) {  // 새 항목 삽입
        if (idx.count(k)) return {false, 0};  // 이미 존재하면 방출 없음
        bool ev = false; uint32_t vic = 0;
        if (q.size() >= cap) {  // 큐가 가득 찼으면 가장 오래된 항목 방출
            vic = q.front(); q.pop_front(); idx.erase(vic); ev = true;
        }
        q.push_back(k); idx[k] = true; return {ev, vic};
    }
    void touch(uint32_t) {}  // 접근 시 아무 동작도 하지 않음
private:
    size_t cap;  // 캐시 용량
    deque<uint32_t> q;  // FIFO 큐
    unordered_map<uint32_t, bool> idx;  // 항목 존재 여부
};

// LRU (Least Recently Used) 정책 구현
struct LRU {
    explicit LRU(size_t c) : cap(c) {}  // 생성자
    bool access(uint32_t k) {  // 항목 접근
        if (!pos.count(k)) return false;  // 존재하지 않으면 false
        lst.erase(pos[k]); lst.push_back(k); pos[k] = prev(lst.end());  // 접근한 항목을 맨 뒤로
        return true;
    }
    pair<bool, uint32_t> insert(uint32_t k) {  // 새 항목 삽입
        if (access(k)) return {false, 0};  // 이미 존재하면 방출 없음
        bool ev = false; uint32_t vic = 0;
        if (lst.size() >= cap) {  // 리스트가 가득 찼으면 가장 오래된 항목 방출
            vic = lst.front(); lst.pop_front(); pos.erase(vic); ev = true;
        }
        lst.push_back(k); pos[k] = prev(lst.end()); return {ev, vic};
    }
    void touch(uint32_t k) { access(k); }  // 접근 시 LRU 업데이트
private:
    size_t cap;  // 캐시 용량
    deque<uint32_t> lst;  // LRU 리스트
    unordered_map<uint32_t, deque<uint32_t>::iterator> pos;  // 항목 위치
};

// LFU (Least Frequently Used) 정책 구현
struct LFU {
    explicit LFU(size_t c) : cap(c), counter(0) {}  // 생성자
    bool access(uint32_t k) {  // 항목 접근
        if (!freq.count(k)) return false;  // 존재하지 않으면 false
        ++freq[k];  // 접근 빈도 증가
        return true;
    }
    pair<bool, uint32_t> insert(uint32_t k) {  // 새 항목 삽입
        if (access(k)) return {false, 0};  // 이미 존재하면 방출 없음
        bool ev = false; uint32_t vic = 0;
        if (freq.size() >= cap) {  // 캐시가 가득 찼으면
            // 최소 빈도 찾기
            int min_freq = INT_MAX;
            for (const auto& p : freq) {
                if (p.second < min_freq) min_freq = p.second;
            }
            // 최소 빈도 중 가장 오래된 항목 방출
            uint32_t earliest_order = UINT32_MAX;
            for (const auto& p : freq) {
                if (p.second == min_freq && order[p.first] < earliest_order) {
                    vic = p.first;
                    earliest_order = order[p.first];
                }
            }
            freq.erase(vic);
            order.erase(vic);
            ev = true;
        }
        freq[k] = 1;  // 새 항목의 빈도는 1
        order[k] = counter++;  // 삽입 순서 기록
        return {ev, vic};
    }
    void touch(uint32_t k) { access(k); }  // 접근 시 빈도 증가
private:
    size_t cap;  // 캐시 용량
    unordered_map<uint32_t, int> freq;  // 접근 빈도
    unordered_map<uint32_t, uint32_t> order;  // 삽입 순서
    uint32_t counter;  // 삽입 카운터
};

}

// 기본 시뮬레이터 템플릿 클래스
template<typename Policy>
class BasicSimulator {
public:
    // 생성자: 프레임 수와 TLB 크기로 초기화
    BasicSimulator(int nframes, int tlbsz)
        : pt(1 << LEVEL1_BITS, vector<PageTableEntry>(1 << LEVEL2_BITS)), tlb(tlbsz), frame(nframes), frames(nframes, UINT32_MAX) {}

    // 메모리 접근 처리
    void access_memory(uint32_t vaddr)
    {
        ++refs;  // 참조 횟수 증가
        uint32_t vpn = vaddr >> PAGE_OFFSET_BITS;  // 가상 페이지 번호
        uint32_t off = vaddr & OFFSET_MASK;  // 페이지 내 오프셋
        uint32_t l1 = (vpn >> LEVEL2_BITS) & LEVEL1_MASK;  // 1단계 인덱스
        uint32_t l2 =  vpn & LEVEL2_MASK;  // 2단계 인덱스

        bool tlb_hit = tlb.access(vpn);  // TLB 검색
        uint32_t pfn = 0; bool pg = false; bool ev = false; uint32_t vic = 0;

        if (tlb_hit) {  // TLB 히트
            ++tlb_hits; pfn = tlb_map[vpn]; tlb.touch(vpn); frame.touch(vpn);
        } else {  // TLB 미스
            ++tlb_miss;
            if (!pt[l1][l2].valid) {  // 페이지 폴트
                pg = true; ++page_faults;
                size_t f = find_free_frame();  // 사용 가능한 프레임 찾기
                if (f == SIZE_MAX) {  // 프레임이 없으면 교체
                    auto [e, v] = frame.insert(vpn); ev = e; vic = v; f = vpn2frame[v];
                    invalidate(v);
                } else {
                    frame.insert(vpn);
                }
                occupy(f, vpn);  // 프레임 할당
                pt[l1][l2] = {static_cast<uint32_t>(f), true};  // 페이지 테이블 업데이트
                pfn = static_cast<uint32_t>(f);
            } else {
                pfn = pt[l1][l2].ppn; frame.touch(vpn);
            }
            auto [tev, oldvpn] = tlb.insert(vpn);  // TLB 업데이트
            if (tev) tlb_map.erase(oldvpn);
            tlb_map[vpn] = pfn;
        }

        uint32_t paddr = (pfn << PAGE_OFFSET_BITS) | off;  // 물리 주소 계산
        print_hex32(vaddr); cout << " -> "; print_hex32(paddr);  // 결과 출력
        cout << ", TLB " << (tlb_hit ? "hit" : "miss")
             << ", "<< (pg ? "Page fault" : "No page fault");
        if (ev) { cout << ", Evicted "; print_hex32(vic << PAGE_OFFSET_BITS); }
        cout << '\n' << dec;
    }

    // 통계 정보 출력
    void print_statistics() const
    {
        cout << "Total references: " << refs << '\n'
             << "TLB hits: " << tlb_hits << '\n'
             << "TLB misses: " << tlb_miss << '\n'
             << "TLB hit ratio: " << fixed << setprecision(1)
             << (refs ? tlb_hits * 100.0 / refs : 0.0) << "%\n"
             << "Page faults: " << page_faults << '\n'
             << "Page fault rate: " << fixed << setprecision(1)
             << (refs ? page_faults * 100.0 / refs : 0.0) << "%\n";
    }

private:
    // 사용 가능한 프레임 찾기
    size_t find_free_frame() const {
        for (size_t i = 0; i < frames.size(); ++i)
            if (frames[i] == UINT32_MAX) return i;
        return SIZE_MAX;
    }

    // 프레임 할당
    void occupy(size_t f, uint32_t vpn) {
        frames[f] = vpn; vpn2frame[vpn] = static_cast<uint32_t>(f);
    }

    // 페이지 무효화
    void invalidate(uint32_t victim) {
        size_t f = vpn2frame[victim]; frames[f] = UINT32_MAX; vpn2frame.erase(victim);
        uint32_t e1 = (victim >> LEVEL2_BITS) & LEVEL1_MASK;
        uint32_t e2 =  victim & LEVEL2_MASK;
        pt[e1][e2].valid = false; tlb_map.erase(victim);
    }

    vector<vector<PageTableEntry>> pt;  // 페이지 테이블
    Policy tlb, frame;  // TLB와 프레임 교체 정책
    unordered_map<uint32_t, uint32_t> tlb_map, vpn2frame;  // VPN -> PPN 매핑
    vector<uint32_t> frames;  // 물리 프레임

    uint64_t refs = 0, tlb_hits = 0, tlb_miss = 0, page_faults = 0;  // 통계 정보
};

// 정책별 시뮬레이터 타입 정의
using FIFOSim = BasicSimulator<policy::FIFO>;
using LRUSim  = BasicSimulator<policy::LRU>;
using LFUSim  = BasicSimulator<policy::LFU>;

// 메인 함수
int main(int argc, char *argv[])
{
    // 명령행 인자 검사
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <num_frames> <tlb_size> <FIFO|LRU|LFU>\n";
        return 1;
    }

    // 명령행 인자 파싱
    int nframes = stoi(argv[1]);  // 프레임 수
    int tlbsz   = stoi(argv[2]);  // TLB 크기
    string alg  = argv[3];        // 페이지 교체 알고리즘
    transform(alg.begin(), alg.end(), alg.begin(), ::toupper);  // 대문자 변환

    // 알고리즘에 따른 시뮬레이터 선택 및 실행
    if (alg == "S3FIFO") {
        VMSimulator simulator(nframes, tlbsz);
        
        string line;
        while (getline(cin, line)) {
            uint32_t vaddr = stoul(line, nullptr, 16);
            simulator.access_memory(vaddr);
        }
        
        simulator.print_statistics();
    } else if (alg == "FIFO") {
        FIFOSim sim(nframes, tlbsz); string line;
        while (getline(cin, line)) if (!line.empty()) sim.access_memory(static_cast<uint32_t>(stoul(line, nullptr, 16)));
        sim.print_statistics();
    } else if (alg == "LRU") {
        LRUSim sim(nframes, tlbsz); string line;
        while (getline(cin, line)) if (!line.empty()) sim.access_memory(static_cast<uint32_t>(stoul(line, nullptr, 16)));
        sim.print_statistics();
    } else if (alg == "LFU") {
        LFUSim sim(nframes, tlbsz); string line;
        while (getline(cin, line)) if (!line.empty()) sim.access_memory(static_cast<uint32_t>(stoul(line, nullptr, 16)));
        sim.print_statistics();
    } else {
        cerr << "Unsupported algorithm: " << alg << '\n';
        return 1;
    }
    return 0;
}