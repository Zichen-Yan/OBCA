/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: CurrentInfo.c
  Source: CurrentInfo.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "CurrentInfo.h"

static const uint32_t byd_interfaces_msg_CurrentInfo_ops [] =
{
  /* CurrentInfo */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_CurrentInfo, ID),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P0_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P0_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P1_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P1_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P2_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P2_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P3_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, P3_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_CurrentInfo, Width),
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_FP, offsetof (byd_interfaces_msg_CurrentInfo, Angle),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_CurrentInfo, ParkingSpaceAvailable),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_CurrentInfo, ParkingSpaceType),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_CurrentInfo_desc =
{
  .m_size = sizeof (byd_interfaces_msg_CurrentInfo),
  .m_align = 8u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::CurrentInfo_",
  .m_keys = NULL,
  .m_nops = 14,
  .m_ops = byd_interfaces_msg_CurrentInfo_ops,
  .m_meta = ""
};

