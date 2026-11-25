#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
#include "mini_v1_protocol.hpp"

namespace tianbot_core::mini_v1_5 {
/* 保留每个扇区全部点；只发布扇区连续覆盖 360° 的整圈，各扇区点数允许变化。
 * 不以 10 Hz 定时器重发旧扫描，不把缺扇区补成完整圈。 */
class LidarScanAssembler {
public:
    struct Scan {
        std::vector<float> ranges, intensities, angles;
        double duration_s{};
        float angle_min{}, angle_increment{};
    };
    uint64_t frames{}, points{}, bad{}, incomplete{}, completed{}, sequence_gaps{};
    bool push(const Frame &f, double local_s, Scan &out) {
        const auto &p = f.payload;
        if (p.size() < 13 || p[0] != 0xcf || p[1] != 0xfa) { ++bad; return false; }
        const auto n = read_u16(p.data()+2), start = read_u16(p.data()+4), span = read_u16(p.data()+6);
        if (!n || n > 240 || p.size() != 10U+3U*n || start >= 3600 || !span || span > 3600) { ++bad; return false; }
        uint16_t sum = n+start+span;
        for (unsigned i=0;i<n;++i) sum += p[8+3*i]+read_u16(p.data()+9+3*i);
        if (sum != read_u16(p.data()+p.size()-2)) { ++bad; return false; }
        ++frames; points += n;
        // 该序号只用于原始帧流；重启/乱序也记异常，不直接声称丢包数量。
        if (have_sequence_ && f.sequence != last_sequence_+1U) { ++sequence_gaps; broken_=true; }
        have_sequence_=true; last_sequence_=f.sequence;
        bool ready=false;
        // 当前雷达按固定扇区、起始角递增发包；首圈必须从 0°开始。
        if (start == 0) {
            if (collecting_) {
                if (!broken_ && expected_ == 3600 && size_ && local_s>began_) {
                    out.ranges.assign(ranges_.begin(),ranges_.begin()+size_);
                    out.intensities.assign(intensities_.begin(),intensities_.begin()+size_);
                    out.angles.assign(angles_.begin(),angles_.begin()+size_);
                    out.angle_min=0;
                    out.angle_increment=static_cast<float>(step_*3.141592653589793/1800.0);
                    out.duration_s=local_s-began_;
                    ++completed; ready=true;
                } else ++incomplete;
            }
            collecting_=true; broken_=false; expected_=0; size_=0; step_=double(span)/n; began_=local_s;
        }
        if (!collecting_) return ready;
        if (start!=expected_ || size_+n>ranges_.size()) broken_=true;
        if (size_+n<=ranges_.size()) {
            for (unsigned i=0;i<n;++i) {
                const auto mm=read_u16(p.data()+9+3*i);
                angles_[size_]=static_cast<float>((start+double(span)*i/n)*3.141592653589793/1800.0);
                ranges_[size_]=mm?mm/1000.0f:INFINITY;
                intensities_[size_]=p[8+3*i]; ++size_;
            }
        }
        expected_=unsigned(start)+span;
        return ready;
    }
private:
    std::array<float,4096> ranges_{}, intensities_{}, angles_{};
    unsigned expected_{}, size_{};
    double step_{}, began_{};
    bool collecting_{}, broken_{}, have_sequence_{};
    uint32_t last_sequence_{};
};
}
