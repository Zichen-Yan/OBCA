/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  The generate program idlc named byd_idl has been modified by BYD
  File name: Freespacecell_rt.c
  Source: Freespacecell_rt.idl
  Cyclone DDS: V0.9.1

*****************************************************************/
#include "Freespacecell_rt.h"

static const uint32_t uss_msgs_msg_FreeSpaceCell_RT_ops [] =
{
  /* FreeSpaceCell_RT */
  DDS_OP_ADR | DDS_OP_TYPE_1BY, offsetof (uss_msgs_msg_FreeSpaceCell_RT, Status),
  DDS_OP_ADR | DDS_OP_TYPE_2BY, offsetof (uss_msgs_msg_FreeSpaceCell_RT, Probability),
  DDS_OP_RTS
};

const dds_topic_descriptor_t uss_msgs_msg_FreeSpaceCell_RT_desc =
{
  .m_size = sizeof (uss_msgs_msg_FreeSpaceCell_RT),
  .m_align = 2u,
  .m_flagset = DDS_TOPIC_FIXED_SIZE,
  .m_nkeys = 0u,
  .m_typename = "uss_msgs::msg::dds_::FreeSpaceCell_RT_",
  .m_keys = NULL,
  .m_nops = 3,
  .m_ops = uss_msgs_msg_FreeSpaceCell_RT_ops,
  .m_meta = ""
};

