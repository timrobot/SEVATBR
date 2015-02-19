#ifndef comm_h
#define comm_h

#ifdef __cplusplus
extern "C" {
#endif

int comm_init(void);
void comm_update(void);
void comm_flush(void);
void comm_destroy(void);

#ifdef __cplusplus
}
#endif

#endif
