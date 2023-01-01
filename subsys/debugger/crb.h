
#ifndef _DEBUGGER_CRB_H_
#define _DEBUGGER_CRB_H_

/* command response buffer */
struct crb {
    uint8_t buf[CONFIG_DEBUGGER_CRB_BUF_SIZE];
    int len;
};

struct crb_queue {
    struct crb crbs[CONFIG_DEBUGGER_CRB_BUF_COUNT];
    int ntu;    /* next to use */
    int ntc;    /* next to clean */
};

extern struct crb *cmd_crb;
extern struct crb_queue *resp_queue;

static inline struct crb *crb_queue_claim(struct crb_queue *q)
{
    if (q->ntu - q->ntc < CONFIG_DEBUGGER_CRB_BUF_COUNT - 1) {
        return q->crbs[q->ntu++ % CONFIG_DEBUGGER_CRB_BUF_COUNT];
    } else {
        return NULL;
    }
}

static inline struct crb *crb_queue_get_resp(struct crb_queue *q)
{
    if (q->ntc < q->ntu) {
        return q->crbs[q->ntc++];
    } else {
        return NULL;
    }
}

int crb_init(void);

int debugger_handle_cmd(struct crb *cmd, struct crb *resp);

#endif /* _DEBUGGER_CRB_H_ */
