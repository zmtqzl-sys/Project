第10周：Docker 概念与 OpenCV 实验 课堂笔记

 Docker 核心
- 本质：容器化技术，打包环境与应用 → 一次构建，到处运行
- 镜像 Image：只读模板，含所有依赖，像“安装包/类”
- 容器 Container：镜像的运行实例，加了可写层，像“程序/实例”
- 常用命令
  - `pull` 拉镜像、`run` 启容器、`ps` 看状态
  - `stop/rm` 停/删容器、`rmi` 删镜像、`build` 建镜像
- 目录挂载：`-v 本地目录:容器目录` → 双向同步文件
- 端口映射：`-p 本地端口:容器端口` → 外部访问容器服务

 OpenCV 核心
- 简介：开源视觉库，跨平台，免费商用，支持 Python/C++
- 安装：`pip install opencv-python opencv-contrib-python`
- 基础操作
  - 读图：`cv2.imread()`（默认BGR格式）
  - 转色彩：`cv2.cvtColor()` → BGR↔RGB/灰度
  - 显示：配合 matplotlib 显示，注意色彩转换The file is too long. Dola only read the first 4%.
