/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: TopMachine.c
  Source: TopMachine.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "TopMachine.h"

static const uint32_t byd_interfaces_msg_topM_ops [] =
{
  /* topM */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (byd_interfaces_msg_topM, top_proce_sts),
  DDS_OP_ADR | DDS_OP_TYPE_ARR | DDS_OP_SUBTYPE_1BY, offsetof (byd_interfaces_msg_topM, padding), 7u,
  DDS_OP_RTS
};

const dds_topic_descriptor_t byd_interfaces_msg_topM_desc =
{
  .m_size = sizeof (byd_interfaces_msg_topM),
  .m_align = 1u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "byd_interfaces::msg::dds_::topM_",
  .m_keys = NULL,
  .m_nops = 3,
  .m_ops = byd_interfaces_msg_topM_ops,
  .m_meta = ""
};

