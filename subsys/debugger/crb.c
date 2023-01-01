
#include "crb.h"

struct crb_queue _crb_queue;
struct crb _cmd_crb;

int crb_init(void)
{
    resp_queue = &_crb_queue;
    cmd_crb = &_cmd_crb;

    return 0;
}
