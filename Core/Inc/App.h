#ifndef APP_H
#define APP_H

/* Each firmware links exactly one implementation of these hooks. */
void App_Init(void);
void App_RunOnce(void);

#endif /* APP_H */
