/*
 * state_machine.c - 路径状态机实现
 *
 * 场地几何 (3-4-5三角形):
 *   A-B 水平距离 60cm
 *   C-D 水平距离 60cm
 *   A-C 垂直距离 80cm  (对角线 ≈100cm)
 *   B-D 垂直距离 80cm  (对角线 ≈100cm)
 *   弧线 BC ≈125.7cm (右半圆弧, 半径40cm)
 *   弧线 DA ≈125.7cm (左半圆弧, 半径40cm)
 *
 * 顶点检测规则:
 *   盲走段 → 传感器检测到黑线 = 到达顶点
 *   循迹段 → 传感器检测不到黑线 = 到达顶点
 *   距离需在目标距离的 ±10% 以内 (或超时 1.4x 强制到达)
 */

#include "state_machine.h"
#include "sensor.h"
#include "encoder.h"
#include <math.h>

/* ---- 场地参数 (cm) 2026H题 ---- */
#define DIST_AB     150.0f    /* AB=CD=1.5m */
#define DIST_ARC    157.1f    /* π×0.5m 半圆弧 */

/* ---- 顶点检测容差 ---- */

/* ---- 内部状态 ---- */
static uint8_t   g_task      = 0;
static uint8_t   g_seg_idx   = 0;
static uint8_t   g_lap       = 0;
static uint8_t   g_lap_total = 1;
static Vertex_t  g_current_v = VERTEX_NONE;
static Segment_t g_segments[12];
static uint8_t   g_seg_count = 0;
static float     g_seg_start = 0.0f;    /* 当前段起始距禈 */

/* ================================================================
 *  预定义路径 (按任务)
 * ================================================================ */

/* 2026H题: 全循迹闭环 A→B(150cm)→弧BC(157cm)→C→D(150cm)→弧DA(157cm)→A */
static const Segment_t path_task2[] = {
    {SEG_LINE_BC,   0.0f,   DIST_AB,   VERTEX_B},
    {SEG_LINE_BC,   0.0f,   DIST_ARC,  VERTEX_C},
    {SEG_LINE_DA,   0.0f,   DIST_AB,   VERTEX_D},
    {SEG_LINE_DA,   0.0f,   DIST_ARC,  VERTEX_A},
    {SEG_DONE,      0.0f,   0.0f,      VERTEX_NONE}
};
#define path_task3 path_task2
#define path_task4 path_task2

/* ================================================================
 *  API 实现
 * ================================================================ */

void sm_set_task(uint8_t task_id, Vertex_t start)
{
    g_task      = task_id;
    g_seg_idx   = 0;
    g_lap       = 0;
    g_current_v = start;
    g_lap_total = (task_id == 4) ? 4 : 1;
    g_seg_start = encoder_get_distance();
    const Segment_t *src;
    uint8_t cnt;

    switch (task_id) {
        case 1: return;  /* 任务1: main.c中单独处理 */
        case 2:
        case 3:
        case 4:
            src = path_task2; cnt = 4; break;
        default:
            return;
    }

    for (uint8_t i = 0; i < cnt; i++) {
        g_segments[i] = src[i];
    }
    g_seg_count = cnt;
}

Segment_t sm_get_current_segment(void)
{
    if (g_seg_idx >= g_seg_count) {
        Segment_t done = {SEG_DONE, 0.0f, 0.0f, VERTEX_NONE};
        return done;
    }
    return g_segments[g_seg_idx];
}

uint8_t sm_get_task(void)  { return g_task; }
bool    sm_is_done(void)   { return g_seg_idx >= g_seg_count; }
uint8_t sm_get_lap(void)   { return g_lap; }

void sm_sync_start(void)
{
    g_seg_start = encoder_get_distance();
}

/*
 * 顶点检测:
 *   盲走段: 距禮>5cm 且检测到线 → 到达
 *   循迹段: 距禮过半 且全白 → 到达
 *   超时保护: 距禮 > 目标*1.4 强制到达
 */
Vertex_t sm_check_vertex(float distance, bool line_detected,
                         float target_dist)
{
    float seg_dist = distance - g_seg_start;  /* 当前段已走距禈 */
    SegType_t type = g_segments[g_seg_idx].type;
    bool arrived = false;

    if (type == SEG_LINE_BC || type == SEG_LINE_DA) {
        if (!line_detected && seg_dist > target_dist * 0.25f) arrived = true;
    }

    if (seg_dist > target_dist * 1.4f) {
        arrived = true;
    }

    if (arrived) {
        return g_segments[g_seg_idx].destination;
    }
    return VERTEX_NONE;
}

/* 到达顶点: 切换下一段, 任务3/4 处理多圈 */
void sm_vertex_reached(Vertex_t v)
{
    g_current_v = v;
    g_seg_idx++;
    g_seg_start = encoder_get_distance();  /* 记录新段起点 */

    if (g_seg_idx >= g_seg_count) {
        g_lap++;
        if (g_lap < g_lap_total) {
            g_seg_idx = 0;
            g_seg_start = encoder_get_distance();
        }
    }
}

/* 盲走纠偏: 走了预期 1.3 倍还没到, 触发扇形搜索 (当前版本已不需要) */
bool sm_needs_recovery(float distance, float target_dist)
{
    Segment_t seg = g_segments[g_seg_idx];
    if (seg.type != SEG_BLIND) return false;
    return (distance > target_dist * 1.3f);
}
