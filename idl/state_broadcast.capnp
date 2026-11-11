@0x853a8a956fc73e4c;

# /sentry/state_broadcast 消息体（结构化，无堡垒/无对外威胁枚举）

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("senseAD::msg::sentinel");

struct StateBroadcast {
  timestampNs @0 :UInt64;

  # ---- 哨兵状态机 ----
  sentryState @1 :Int32;
  sentryStateName @2 :Text;        # UNAVAILABLE/STANDBY/ACTIVE/WARNING/ALARM

  # ---- 实时威胁评估 ----
  threatLevel @3 :Int32;
  threatLevelName @4 :Text;        # NONE/LOW/MEDIUM/HIGH
  rawThreatId @5 :Int32;
  smThreatId @6 :Int32;
  localTrackId @7 :Int32;          # rawThreatId % 100000
  threatCamera @8 :Text;           # front/rear/left/right_camera_fov195 或 imu/-

  targetCount @9 :Int32;
  obstacleType @10 :Int32;
  obstacleTypeName @11 :Text;

  # ---- 原因（完整中文 + 短码）----
  reasonCode @12 :Text;
  reason @13 :Text;

  summary @14 :Text;
}
