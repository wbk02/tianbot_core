#pragma once
#include <cstdint>
namespace tianbot_core::mini_v1_5 { struct Session { std::uint32_t id = 0; std::uint32_t lease_deadline_ms = 0; }; bool session_active(const Session &session, std::uint32_t now_ms); }
