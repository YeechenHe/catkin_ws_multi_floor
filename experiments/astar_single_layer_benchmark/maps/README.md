# 地图目录说明

本目录存放单层实验地图。

建议输出文件命名：

- `single_layer_eval.pgm`
- `single_layer_eval.yaml`

其中地图应包含三类区域：

1. 狭窄区域（narrow）
2. 常规区域（normal）
3. 宽阔区域（open）

`scripts/build_synthetic_map.py` 会根据场景模板生成一版基础地图，后续可以再手工微调。
