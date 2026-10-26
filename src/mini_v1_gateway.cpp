#include "tianbot_core/mini_v1_gateway.hpp"
#include <algorithm>
namespace tianbot_core { namespace mini_v1_5 {
std::uint16_t read_u16(const std::uint8_t*p){return std::uint16_t(p[0])|std::uint16_t(p[1])<<8;} std::uint32_t read_u32(const std::uint8_t*p){return std::uint32_t(p[0])|std::uint32_t(p[1])<<8|std::uint32_t(p[2])<<16|std::uint32_t(p[3])<<24;} void write_u16(std::uint8_t*p,std::uint16_t v){p[0]=v;p[1]=v>>8;} void write_u32(std::uint8_t*p,std::uint32_t v){p[0]=v;p[1]=v>>8;p[2]=v>>16;p[3]=v>>24;}
std::uint32_t mini_crc32(const std::uint8_t*d,std::size_t n){std::uint32_t c=0xffffffffU;for(std::size_t i=0;i<n;i++){c^=d[i];for(int b=0;b<8;b++)c=(c>>1)^((c&1U)?0xedb88320U:0);}return c^0xffffffffU;}
bool decode_mini_frame(const std::uint8_t*d,std::size_t n,Frame*f){if(!d||!f||n<16||read_u16(d)!=0x5442||d[2]!=1||d[5]!=12)return false;auto l=read_u16(d+10);if(l>1200||n!=16+l||read_u32(d+12+l)!=mini_crc32(d,12+l))return false;f->type=d[3];f->flags=d[4];f->sequence=read_u32(d+6);f->payload.assign(d+12,d+12+l);return true;}
bool encode_mini_frame(std::uint8_t t,std::uint8_t fl,std::uint32_t seq,const std::vector<std::uint8_t>&p,std::vector<std::uint8_t>*o){if(!o||p.size()>1200)return false;o->assign(16+p.size(),0);auto*d=o->data();write_u16(d,0x5442);d[2]=1;d[3]=t;d[4]=fl;d[5]=12;write_u32(d+6,seq);write_u16(d+10,p.size());std::copy(p.begin(),p.end(),d+12);write_u32(d+12+p.size(),mini_crc32(d,12+p.size()));return true;}
} }
