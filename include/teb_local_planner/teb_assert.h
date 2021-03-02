#define TEB_ASSERT_MSG(cond, ...) \
 if (!cond) \
 { \
    printf(__VA_ARGS__); \
    assert(false); \
 }
