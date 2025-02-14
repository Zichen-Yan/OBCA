/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: fusion_parkinginfo.c
  Source: fusion_parkinginfo.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "fusion_parkinginfo.h"

static const uint32_t byd_interfaces_msg_fusion_parkinginfo_ops [] =
{
  /* fusion_parkinginfo */
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_fusion_parkinginfo, TimeStampMs),
  DDS_OP_ADR | DDS_OP_TYPE_EXT, offsetof (byd_interfaces_msg_fusion_parkinginfo, position), (3u << 16u) + 17u /* Position */,
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_STU, offsetof (byd_interfaces_msg_fusion_parkinginfo, parkingSpaceInfo), 25u, (5u << 16u) + 21u /* ParkingSpaceInfo */, sizeof (uss_msgs_msg_ParkingSpaceInfo),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_fusion_parkinginfo, TraceParkingID_Cam),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_fusion_parkinginfo, TraceParkingID_USS),
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (byd_interfaces_msg_fusion_parkinginfo, Theta),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_fusion_parkinginfo, ParkInMode),
  DDS_OP_RTS,

  /* Position */
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (uss_msgs_msg_Position, X),
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (uss_msgs_msg_Position, Y),
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (uss_msgs_msg_Position, Heading),
  DDS_OP_RTS,

  /* ParkingSpaceInfo */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P0_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P0_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P1_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P1_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P2_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P2_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P3_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P3_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P0_X_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P0_Y_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P1_X_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P1_Y_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P2_X_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P2_Y_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P3_X_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, P3_Y_Offset),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, Width),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, ParkingSpaceValid),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (uss_msgs_msg_ParkingSpaceInfo, ParkingSpaceType),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_fusion_parkinginfo_desc =
{
  .m_size = sizeof (byd_interfaces_msg_fusion_parkinginfo),
  .m_align = 8u,
  .m_flagset = DDS_TOPIC_NO_OPTIMIZE | DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::fusion_parkinginfo_",
  .m_keys = NULL,
  .m_nops = 33,
  .m_ops = byd_interfaces_msg_fusion_parkinginfo_ops,
  .m_meta = ""
};

