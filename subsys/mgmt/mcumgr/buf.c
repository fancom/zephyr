/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "net/buf.h"
#include "mgmt/mcumgr/buf.h"

NET_BUF_POOL_DEFINE(pkt_pool, CONFIG_MCUMGR_BUF_COUNT, CONFIG_MCUMGR_BUF_SIZE,
		    CONFIG_MCUMGR_BUF_USER_DATA_SIZE, NULL);

struct net_buf *
mcumgr_buf_alloc(void)
{
	return net_buf_alloc(&pkt_pool, K_NO_WAIT);
}

void
mcumgr_buf_free(struct net_buf *nb)
{
	net_buf_unref(nb);
}

void
cbor_nb_reader_init(struct cbor_nb_reader *cnr,
		    struct net_buf *nb)
{
	cnr->nb = nb;
	/* here we give the actual size of the encoded data to the parser, so nb->len */
	cbor_parser_init(nb->data, nb->len, 0, &cnr->parser, &cnr->it);
}

void
cbor_nb_writer_init(struct cbor_nb_writer *cnw, struct net_buf *nb)
{
	cnw->nb = nb;
	/* here we give the max size of the encoding buffer to the encoder, so nb->size */
	cbor_encoder_init(&cnw->encoder, nb->data, nb->size, 0);
}
