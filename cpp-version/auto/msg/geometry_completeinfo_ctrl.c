/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: geometry_completeinfo_ctrl.c
  Source: geometry_completeinfo_ctrl.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "geometry_completeinfo_ctrl.h"

static const uint32_t byd_interfaces_msg_geometry_completeinfo_ctrl_ops [] =
{
  /* geometry_completeinfo_ctrl */
  DDS_OP_ADR | DDS_OP_TYPE_8BY | DDS_OP_FLAG_SGN, offsetof (byd_interfaces_msg_geometry_completeinfo_ctrl, TimeStampMs),
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_geometry_completeinfo_ctrl, IsPlanningCompleted),
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_geometry_completeinfo_ctrl_desc =
{
  .m_size = sizeof (byd_interfaces_msg_geometry_completeinfo_ctrl),
  .m_align = 8u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::geometry_completeinfo_ctrl_",
  .m_keys = NULL,
  .m_nops = 3,
  .m_ops = byd_interfaces_msg_geometry_completeinfo_ctrl_ops,
  .m_meta = ""
};

