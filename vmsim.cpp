// -----------------------------------------------------------------------------
// Virtual Memory Simulator supporting FIFO, LRU, LFU *in their own classes*.
// Original S3-FIFO-based VMSimulator remains essentially untouched.
// -----------------------------------------------------------------------------
// Build : g++ -std=c++17 -O2 -Wall -Wextra -pedantic -o vmsim vmsim.cpp
// Usage : ./vmsim <num_frames> <tlb_size> <FIFO|LRU|LFU|S3FIFO>
// -----------------------------------------------------------------------------

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

/* ─────────────────────── Address-size constants ─────────────────────── */
constexpr int  PAGE_OFFSET_BITS = 12;                    // 4 KiB pages
constexpr int  LEVEL1_BITS      = 10;
constexpr int  LEVEL2_BITS      = 10;
constexpr uint32_t LEVEL1_MASK  = (1u << LEVEL1_BITS) - 1;
constexpr uint32_t LEVEL2_MASK  = (1u << LEVEL2_BITS) - 1;
constexpr uint32_t OFFSET_MASK  = (1u << PAGE_OFFSET_BITS) - 1;

/* ─────────────────────── Page-table helper ──────────────────────────── */
struct PageTableEntry { uint32_t ppn = 0; bool valid = false; };

/* -----------------------------------------------------------------------------
 *                          Original S3-FIFO structures
 * -------------------------------------------------------------------------- */
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
        for(size_t i=0; i<frames.size(); i++){
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

static inline void print_hex32(uint32_t v)
{
    cout << "0x" << uppercase << hex << setw(8) << setfill('0') << v;
}


/* ───────────────────── Replacement‑policy classes ─────────────────── */
namespace policy {

struct FIFO {
    explicit FIFO(size_t c) : cap(c) {}
    bool access(uint32_t k) { return idx.count(k); }
    pair<bool, uint32_t> insert(uint32_t k) {
        if (idx.count(k)) return {false, 0};
        bool ev = false; uint32_t vic = 0;
        if (q.size() >= cap) { vic = q.front(); q.pop_front(); idx.erase(vic); ev = true; }
        q.push_back(k); idx[k] = true; return {ev, vic};
    }
    void touch(uint32_t) {}
private:
    size_t cap; deque<uint32_t> q; unordered_map<uint32_t, bool> idx;
};

struct LRU {
    explicit LRU(size_t c) : cap(c) {}
    bool access(uint32_t k) {
        if (!pos.count(k)) return false;
        lst.erase(pos[k]); lst.push_back(k); pos[k] = prev(lst.end());
        return true;
    }
    pair<bool, uint32_t> insert(uint32_t k) {
        if (access(k)) return {false, 0};
        bool ev = false; uint32_t vic = 0;
        if (lst.size() >= cap) { vic = lst.front(); lst.pop_front(); pos.erase(vic); ev = true; }
        lst.push_back(k); pos[k] = prev(lst.end()); return {ev, vic};
    }
    void touch(uint32_t k) { access(k); }
private:
    size_t cap; deque<uint32_t> lst; unordered_map<uint32_t, deque<uint32_t>::iterator> pos;
};

struct LFU {
    explicit LFU(size_t c) : cap(c), counter(0) {}
    bool access(uint32_t k) { 
        if (!freq.count(k)) return false; 
        ++freq[k]; 
        return true; 
    }
    pair<bool, uint32_t> insert(uint32_t k) {
        if (access(k)) return {false, 0};
        bool ev = false; uint32_t vic = 0;
        if (freq.size() >= cap) {
            // Find minimum frequency
            int min_freq = INT_MAX;
            for (const auto& p : freq) {
                if (p.second < min_freq) min_freq = p.second;
            }
            // Among items with min frequency, evict the one with earliest insertion order
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
        freq[k] = 1;
        order[k] = counter++;
        return {ev, vic};
    }
    void touch(uint32_t k) { access(k); }
private:
    size_t cap;
    unordered_map<uint32_t, int> freq;
    unordered_map<uint32_t, uint32_t> order;
    uint32_t counter;
};

} // namespace policy


/* ───────────────────── Generic VM simulator ───────────────────── */

template<typename Policy>
class BasicSimulator {
public:
    BasicSimulator(int nframes, int tlbsz)
        : pt(1 << LEVEL1_BITS, vector<PageTableEntry>(1 << LEVEL2_BITS))
        , tlb(tlbsz)
        , frame(nframes)
        , frames(nframes, UINT32_MAX) {}

    void access_memory(uint32_t vaddr)
    {
        ++refs;
        uint32_t vpn = vaddr >> PAGE_OFFSET_BITS;
        uint32_t off = vaddr & OFFSET_MASK;
        uint32_t l1  = (vpn >> LEVEL2_BITS) & LEVEL1_MASK;
        uint32_t l2  =  vpn & LEVEL2_MASK;

        bool tlb_hit = tlb.access(vpn);
        uint32_t pfn = 0; bool pg = false; bool ev = false; uint32_t vic = 0;

        if (tlb_hit) {
            ++tlb_hits; pfn = tlb_map[vpn]; tlb.touch(vpn); frame.touch(vpn);
        } else {
            ++tlb_miss;
            if (!pt[l1][l2].valid) {
                pg = true; ++page_faults;
                size_t f = find_free_frame();
                if (f == SIZE_MAX) {
                    auto [e, v] = frame.insert(vpn); ev = e; vic = v; f = vpn2frame[v];
                    invalidate(v);
                } else {
                    frame.insert(vpn);
                }
                occupy(f, vpn);
                pt[l1][l2] = {static_cast<uint32_t>(f), true};
                pfn = static_cast<uint32_t>(f);
            } else {
                pfn = pt[l1][l2].ppn; frame.touch(vpn);
            }
            auto [tev, oldvpn] = tlb.insert(vpn);
            if (tev) tlb_map.erase(oldvpn);
            tlb_map[vpn] = pfn;
        }

        uint32_t paddr = (pfn << PAGE_OFFSET_BITS) | off;
        print_hex32(vaddr); cout << " -> "; print_hex32(paddr);
        cout << ", TLB " << (tlb_hit ? "hit" : "miss")
             << ", "      << (pg ? "Page fault" : "No page fault");
        if (ev) { cout << ", Evicted "; print_hex32(vic << PAGE_OFFSET_BITS); }
        cout << '\n' << dec;
    }

    void print_statistics() const
    {
        cout << "Total references: " << refs << '\n'
             << "TLB hits: "        << tlb_hits << '\n'
             << "TLB misses: "      << tlb_miss << '\n'
             << "TLB hit ratio: "   << fixed << setprecision(1)
             << (refs ? tlb_hits * 100.0 / refs : 0.0) << "%\n"
             << "Page faults: "     << page_faults << '\n'
             << "Page fault rate: " << fixed << setprecision(1)
             << (refs ? page_faults * 100.0 / refs : 0.0) << "%\n";
    }

private:
    /* helpers */
    size_t find_free_frame() const {
        for (size_t i = 0; i < frames.size(); ++i)
            if (frames[i] == UINT32_MAX) return i;
        return SIZE_MAX;
    }
    void occupy(size_t f, uint32_t vpn) {
        frames[f] = vpn; vpn2frame[vpn] = static_cast<uint32_t>(f);
    }
    void invalidate(uint32_t victim) {
        size_t f = vpn2frame[victim]; frames[f] = UINT32_MAX; vpn2frame.erase(victim);
        uint32_t e1 = (victim >> LEVEL2_BITS) & LEVEL1_MASK;
        uint32_t e2 =  victim & LEVEL2_MASK;
        pt[e1][e2].valid = false; tlb_map.erase(victim);
    }

    /* structures */
    vector<vector<PageTableEntry>> pt;
    Policy tlb, frame;
    unordered_map<uint32_t, uint32_t> tlb_map, vpn2frame;
    vector<uint32_t> frames;

    /* stats */
    uint64_t refs = 0, tlb_hits = 0, tlb_miss = 0, page_faults = 0;
};

using FIFOSim = BasicSimulator<policy::FIFO>;
using LRUSim  = BasicSimulator<policy::LRU>;
using LFUSim  = BasicSimulator<policy::LFU>;

/* -----------------------------------------------------------------------------
 *                                   main
 * -------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <num_frames> <tlb_size> <FIFO|LRU|LFU>\n";
        return 1;
    }

    int nframes = stoi(argv[1]);
    int tlbsz   = stoi(argv[2]);
    string alg  = argv[3];
    transform(alg.begin(), alg.end(), alg.begin(), ::toupper);

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