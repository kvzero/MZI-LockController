#include <string.h>
#include "app.h"
#include "app_core.h"

AppRuntime_t g_rt;

static const AppPageOps_t *s_page_ops = NULL;
static const AppPageOps_t * const s_page_table[APP_PAGE_COUNT] = {
    &APP_PAGE_WAIT_OPS,
    &APP_PAGE_ACQUIRE_OPS,
    &APP_PAGE_LOCK_OPS,
    &APP_PAGE_FAULT_OPS,
};

void APP_Init(void)
{
    (void)memset(&g_rt, 0, sizeof(g_rt));
    APP_GotoPage(APP_PAGE_WAIT);
}

void APP_Process(void)
{
    if ((s_page_ops != NULL) && (s_page_ops->process != NULL)) {
        s_page_ops->process();
    }
}

void APP_OnButton(Button_Event_t evt)
{
    if ((s_page_ops != NULL) && (s_page_ops->on_button != NULL)) {
        s_page_ops->on_button(evt);
    }
}

void APP_RenderIfNeeded(void)
{
    if (!g_rt.ui_dirty) {
        return;
    }

    if ((s_page_ops != NULL) && (s_page_ops->render != NULL)) {
        s_page_ops->render();
    }

    g_rt.ui_dirty = false;
}

void APP_SetFault(FaultCode_t fault)
{
    if (fault == APP_FAULT_NONE) {
        return;
    }

    if ((g_rt.page == APP_PAGE_FAULT) && (g_rt.fault == fault)) {
        return;
    }

    g_rt.fault = fault;
    APP_GotoPage(APP_PAGE_FAULT);
}

void APP_RequestRender(void)
{
    g_rt.ui_dirty = true;
}

void APP_GotoPage(AppPage_t page)
{
    const AppPageOps_t *next_ops;

    if ((uint32_t)page >= (uint32_t)APP_PAGE_COUNT) {
        return;
    }

    next_ops = s_page_table[page];

    if ((s_page_ops != NULL) && (s_page_ops->exit != NULL)) {
        s_page_ops->exit();
    }

    g_rt.page = page;
    s_page_ops = next_ops;

    if ((s_page_ops != NULL) && (s_page_ops->enter != NULL)) {
        s_page_ops->enter();
    }

    g_rt.ui_dirty = true;
}
