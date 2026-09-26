# rewrite 独立开发目录

本目录从 2026-znc-ss 提取，保留 rewrite 目标程序的编译输入、必要头文件、Loongson 驱动、部署/运行脚本和运行时模型。

编译入口：scripts/build/build_rewrite.sh target
部署入口：scripts/deploy/deploy_rewrite.ps1 -> scripts/deploy/deploy_rewrite.sh
GUI：scripts/gui/start_rewrite_gui.bat

源码按功能位于 `src/`，模型位于 `models/target/`，标定数据位于
`config/`，辅助脚本位于 `scripts/`。

未复制旧版巡线、expt、测试/回放示例和历史资料。
