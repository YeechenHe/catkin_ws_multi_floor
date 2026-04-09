#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# 数据来源说明（重要）：
# - /move_base/global_costmap/costmap：静态地图 + 激光障碍物 + 膨胀，可能与「纯楼层栅格地图」视觉不一致。
# - /map：由 change_map_node 根据当前楼层发布的静态 OccupancyGrid，适合对比 floor0 / floor1。
# 切换楼层后建议：rosservice call /change_map "{req_int: N}"，再运行本脚本；可选 _subscribe_delay_sec 等待 costmap 更新。
#
# 清晰度说明：密度 = 窗口内障碍格占比。R 较大时会“抹匀”墙缘；纯静态 /map 墙往往只有 1 格厚，
# 在固定 vmax=1.0 时整图数值偏小、发灰发糊。static_map 默认 imshow_vmax_mode=stretch 拉伸显示；
# 需要严格 0~1 色标对比时请设 _imshow_vmax_mode:=full。

import rospy
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.axes_grid1 import make_axes_locatable

from nav_msgs.msg import OccupancyGrid


def compute_density(grid, occ_threshold, density_radius):
    height = grid.shape[0]
    width = grid.shape[1]
    grid_clean = grid.copy()
    grid_clean[grid_clean < 0] = 0
    obstacle_mask = (grid_clean >= occ_threshold).astype(np.uint8)

    R = int(density_radius)
    if R <= 0:
        rospy.logerr("density_radius must be positive.")
        return None

    kernel_size = 2 * R + 1
    kernel_area = float(kernel_size * kernel_size)
    padded = np.pad(
        obstacle_mask, ((R, R), (R, R)), mode="constant", constant_values=0
    )
    density = np.zeros_like(obstacle_mask, dtype=np.float32)
    for i in range(height):
        row_end = i + kernel_size
        for j in range(width):
            col_end = j + kernel_size
            window = padded[i:row_end, j:col_end]
            density[i, j] = window.sum() / kernel_area
    return density


def dilate_max_filter(image, radius):
    if radius <= 0:
        return image
    padded = np.pad(image, ((radius, radius), (radius, radius)), mode="edge")
    out = np.zeros_like(image, dtype=np.float32)
    height = image.shape[0]
    width = image.shape[1]
    kernel_size = 2 * radius + 1
    for i in range(height):
        row_end = i + kernel_size
        for j in range(width):
            col_end = j + kernel_size
            out[i, j] = np.max(padded[i:row_end, j:col_end])
    return out


def main():
    rospy.init_node("obstacle_density_visualizer", anonymous=True)

    input_mode = rospy.get_param(
        "~input_mode",
        "global_costmap",
    )
    if input_mode == "static_map":
        default_topic = "/map"
        source_caption = "static /map (change_map)"
    elif input_mode == "global_costmap":
        default_topic = "/move_base/global_costmap/costmap"
        source_caption = "global_costmap (static+obstacles+inflation)"
    else:
        rospy.logerr("input_mode must be 'global_costmap' or 'static_map'.")
        return

    # 兼容旧参数名 ~costmap_topic
    if rospy.has_param("~costmap_topic"):
        topic = rospy.get_param("~costmap_topic")
    else:
        topic = rospy.get_param("~grid_topic", default_topic)
    density_radius = rospy.get_param("~density_radius", 10)
    occ_threshold = rospy.get_param("~occ_threshold", 65)
    output_path = rospy.get_param("~output_path", "obstacle_density_floor0.png")
    map_label = rospy.get_param("~map_label", "")
    fig_width_in = rospy.get_param("~fig_width_in", 16.0)
    fig_height_in = rospy.get_param("~fig_height_in", 16.0)
    save_dpi = rospy.get_param("~save_dpi", 400)
    colormap = rospy.get_param("~colormap", "viridis")
    subscribe_delay_sec = rospy.get_param("~subscribe_delay_sec", 0.0)
    wait_timeout_sec = rospy.get_param("~wait_timeout_sec", 60.0)
    # full: vmin=0 vmax=1（物理密度占比）；stretch：vmax=min(1,max 或高分位)，静态图更醒目
    default_vmax_mode = "stretch" if input_mode == "static_map" else "full"
    imshow_vmax_mode = rospy.get_param("~imshow_vmax_mode", default_vmax_mode)
    stretch_percentile = rospy.get_param("~stretch_percentile", 99.5)
    # 仅用于显示增强（不改变原始 density 统计）
    display_dilate_radius = rospy.get_param("~display_dilate_radius", 1)
    display_gamma = rospy.get_param("~display_gamma", 0.85)
    # 视图布局：自动围绕有效区域居中，减少大面积留白
    auto_center_crop = rospy.get_param("~auto_center_crop", True)
    center_crop_margin_cells = rospy.get_param("~center_crop_margin_cells", 40)
    center_crop_threshold_ratio = rospy.get_param("~center_crop_threshold_ratio", 0.06)
    # 旋转显示方向：0 / 90 / 180 / 270（顺时针）
    rotate_cw_deg = rospy.get_param("~rotate_cw_deg", 0)

    if subscribe_delay_sec > 0.0:
        rospy.loginfo("Sleep %.2f s before capture (map/costmap settle).", subscribe_delay_sec)
        rospy.sleep(subscribe_delay_sec)

    rospy.loginfo(
        "Capturing one message from %s (input_mode=%s, label=%s)",
        topic,
        input_mode,
        map_label,
    )
    try:
        msg = rospy.wait_for_message(topic, OccupancyGrid, timeout=wait_timeout_sec)
    except rospy.ROSException as exc:
        rospy.logerr("Timeout waiting for %s: %s", topic, exc)
        return

    rospy.loginfo(
        "Received: width=%d, height=%d, res=%.3f, frame_id=%s",
        msg.info.width,
        msg.info.height,
        msg.info.resolution,
        msg.header.frame_id,
    )

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    frame_id = msg.header.frame_id

    data = np.array(msg.data, dtype=np.int32)
    if data.size != width * height:
        rospy.logerr(
            "Unexpected data size: %d vs width*height=%d",
            data.size,
            width * height,
        )
        return

    grid = data.reshape((height, width))
    density = compute_density(grid, occ_threshold, density_radius)
    if density is None:
        return

    density_display = dilate_max_filter(density, int(display_dilate_radius))
    if display_gamma <= 0.0:
        rospy.logerr("display_gamma must be positive.")
        return

    R = int(density_radius)
    dmax = float(np.max(density_display))
    if imshow_vmax_mode == "stretch":
        if dmax <= 1e-12:
            vmax_plot = 1.0
            cb_label = "density (empty map fallback vmax=1)"
            rospy.logwarn("Density is all zero; using vmax=1 for display.")
        else:
            high = float(np.percentile(density_display, stretch_percentile))
            vmax_plot = min(1.0, max(dmax, high))
            cb_label = "fraction in window (display vmax=%.4f)" % vmax_plot
            rospy.loginfo(
                "imshow stretch: raw max=%.6f, p%.1f=%.6f -> vmax_plot=%.6f",
                dmax,
                stretch_percentile,
                high,
                vmax_plot,
            )
    elif imshow_vmax_mode == "full":
        vmax_plot = 1.0
        cb_label = "density fraction in window (0~1)"
    else:
        rospy.logerr("imshow_vmax_mode must be 'full' or 'stretch'.")
        return

    density_for_plot = density_display
    if display_gamma != 1.0:
        density_norm = np.clip(density_display / vmax_plot, 0.0, 1.0)
        density_for_plot = np.power(density_norm, display_gamma) * vmax_plot

    rotate_steps = 0
    if rotate_cw_deg in [0, 90, 180, 270]:
        rotate_steps = int(rotate_cw_deg / 90)
    else:
        rospy.logerr("rotate_cw_deg must be one of [0, 90, 180, 270].")
        return
    # np.rot90 是逆时针，顺时针 k 次等价于逆时针 -k 次
    if rotate_steps != 0:
        density_for_plot = np.rot90(density_for_plot, -rotate_steps)

    fig, ax = plt.subplots(figsize=(fig_width_in, fig_height_in), facecolor="white")
    label_part = ""
    if map_label:
        label_part = "[%s] " % map_label
    vmax_note = "vmax=1.0" if imshow_vmax_mode == "full" else ("vmax=%.4f" % vmax_plot)
    ax.set_title(
        "%sObstacle density — %s\nR=%d cells, dilate=%d, gamma=%.2f, rot_cw=%d, res=%.3fm, %dx%d, frame=%s, %s"
        % (
            label_part,
            source_caption,
            R,
            int(display_dilate_radius),
            float(display_gamma),
            int(rotate_cw_deg),
            resolution,
            width,
            height,
            frame_id,
            vmax_note,
        ),
        fontsize=11,
    )
    density_plot = np.flipud(density_for_plot)
    im = ax.imshow(
        density_plot,
        cmap=colormap,
        interpolation="nearest",
        vmin=0.0,
        vmax=vmax_plot,
    )
    if auto_center_crop:
        # 在显示域上做阈值分割并裁剪到有效区域，保留一定边距，改善“地图偏角落/留白过多”
        display_threshold = vmax_plot * float(center_crop_threshold_ratio)
        valid_mask = density_plot >= display_threshold
        if np.any(valid_mask):
            ys, xs = np.where(valid_mask)
            x_min = int(np.min(xs))
            x_max = int(np.max(xs))
            y_min = int(np.min(ys))
            y_max = int(np.max(ys))

            margin = int(center_crop_margin_cells)
            x_min = max(0, x_min - margin)
            x_max = min(width - 1, x_max + margin)
            y_min = max(0, y_min - margin)
            y_max = min(height - 1, y_max + margin)

            # density_plot 已经是显示坐标（包含旋转+flip），直接按显示坐标裁剪
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_max, y_min)
            rospy.loginfo(
                "Auto center crop: x=[%d,%d], y=[%d,%d], margin=%d",
                x_min, x_max, y_min, y_max, margin,
            )
        else:
            rospy.logwarn("Auto center crop skipped: no valid pixels above threshold.")
    # 绑定到主图轴，确保色条与主图上下同高
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="4.5%", pad=0.15)
    cbar = fig.colorbar(im, cax=cax)
    cbar.set_label(cb_label)

    ax.set_xlabel("x (cells)")
    ax.set_ylabel("y (cells)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=save_dpi, facecolor="white")
    rospy.loginfo(
        "Saved to %s (%.1fx%.1f in @ %d dpi)",
        output_path,
        fig_width_in,
        fig_height_in,
        save_dpi,
    )


if __name__ == "__main__":
    main()
