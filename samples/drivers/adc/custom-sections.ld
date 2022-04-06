/*
 * Copyright (c) 2020 Jeroen van Dooren
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <autoconf.h>
#include <linker/sections.h>
#include <devicetree.h>

#include <linker/linker-defs.h>
#include <linker/linker-tool.h>

#ifdef CONFIG_SOC_SERIES_STM32H7X
#if DT_NODE_HAS_STATUS(DT_NODELABEL(sram4), okay)

SECTION_PROLOGUE (shared_sram4, ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(sram4))) (NOLOAD),)
{
    __shared_sram4_start = .;
    KEEP(*(SORT_BY_NAME(".shared_sram4*")))
    __shared_sram4_end = .;
} GROUP_DATA_LINK_IN(SRAM4, SRAM4)

#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sram1), okay)

SECTION_PROLOGUE (shared_sram1, ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(sram1))) (NOLOAD),)
{
    __shared_sram1_start = .;
    KEEP(*(SORT_BY_NAME(".shared_sram1*")))
    __shared_sram1_end = .;
} GROUP_DATA_LINK_IN(SRAM1, SRAM1)

#endif

#endif /* CONFIG_SOC_SERIES_STM32H7X */
