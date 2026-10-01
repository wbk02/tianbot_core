#include "tianbot_core/mini_v1_gateway.hpp"
namespace tianbot_core::mini_v1_5 { bool session_active(const Session &session, std::uint32_t now_ms) { return session.id != 0U && static_cast<std::int32_t>(session.lease_deadline_ms - now_ms) > 0; } }
