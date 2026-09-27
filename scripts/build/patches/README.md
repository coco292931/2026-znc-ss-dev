# 工具链补丁（patches）

## limits.h —— loongson-gnu-toolchain-8.3-i686-mingw rc1.6 缺失文件

### 背景

官方发布的 `loongson-gnu-toolchain-8.3-i686-mingw-loongarch64-linux-gnu-rc1.6.zip`
漏装了 GCC 的内部头文件：

```text
<toolchain>/lib/gcc/loongarch64-linux-gnu/8.3.0/include/limits.h
```

该文件本应定义 ISO C 整数限制宏（`INT_MAX`、`UCHAR_MAX`、`LLONG_MAX`……），
并通过 `#include_next <limits.h>` 链到系统 glibc 的 `limits.h`。
缺失后，glibc `limits.h` 中的

```c
#if defined __GNUC__ && !defined _GCC_LIMITS_H_
# include_next <limits.h>
#endif
```

会继续向后搜索"编译器的 limits.h"而链条断裂，报错：

```text
error: no include path in which to search for limits.h
```

症状：凡引用 `<limits.h>` / `<climits>` / OpenCV 头文件
（如 `opencv2/core/saturate.hpp`）的编译单元全部失败。

### 修复

本目录下的 `limits.h` 是标准的 GCC limits.h 包装版本：
定义 `_GCC_LIMITS_H_` 与全部 ISO 常量后 `#include_next` 到 glibc 版本，
两端协作完整提供 ISO + POSIX 常量。

`scripts/build/build_rewrite.ps1` 在每次构建前会自动检测并补装该文件
（工具链自愈），正常情况下无需手工操作；重新解压工具链 zip 后，
下次构建会自动补回。

### 手工应用（其他工具链 / 其他机器）

```powershell
Copy-Item scripts/build/patches/limits.h `
  "<toolchain>\lib\gcc\loongarch64-linux-gnu\8.3.0\include\limits.h"
```

### 备注

- 同一发行流的 x86_64 Linux 版工具链（`loongson-gnu-toolchain-8.3-x86_64-...`）
  若同样缺失该文件，可用同一补丁修复。
- 该补丁只新增文件，不修改工具链其他内容；若工具链自带的
  `limits.h` 已存在，自愈逻辑不会覆盖。
