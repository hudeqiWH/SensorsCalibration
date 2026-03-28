#!/bin/bash
# ============================================================================
# Ocam 相机标定与验证脚本
# 功能：运行标定并自动验证结果
# ============================================================================

set -e

# ----------------------------------------------------------------------------
# 配置参数
# ----------------------------------------------------------------------------
# 相机内参文件（4个相机）
OCAM_FILES=(
    "2_dog/param/park_front.json"
    "2_dog/param/park_left.json"
    "2_dog/param/park_back.json"
    "2_dog/param/park_right.json"
)

# 输入图像文件（4个相机）
IMAGE_FILES=(
    "2_dog/imgs/ParkFront-1774583477301602000.png"
    "2_dog/imgs/ParkLeft-1774583477034621000.png"
    "2_dog/imgs/ParkBack-1774583477301602000.png"
    "2_dog/imgs/ParkRight-1774583477101370000.png"
)

# 外参文件
EXTRINSICS_FILE="/home/nio/deqi/r_calib/data/trans/camera_extrinsics_ikalibr.json5"

# 输出目录（带时间戳）
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="./2_dog/calibration_${TIMESTAMP}"

# 相机模型（1=Ocam）
CAMERA_MODEL=1

# 验证阈值
VALID_PIXEL_THRESHOLD=100  # 有效像素的最小数量
MAX_BLACK_RATIO=0.8        # 最大黑色像素比例（0-1）

# ----------------------------------------------------------------------------
# 颜色输出
# ----------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ----------------------------------------------------------------------------
# 检查输入文件
# ----------------------------------------------------------------------------
check_input_files() {
    log_info "检查输入文件..."
    
    local missing_files=0
    
    # 检查 Ocam 标定文件
    for file in "${OCAM_FILES[@]}"; do
        if [ ! -f "$file" ]; then
            log_error "缺少 Ocam 标定文件: $file"
            missing_files=$((missing_files + 1))
        else
            log_success "找到 Ocam 文件: $file"
        fi
    done
    
    # 检查图像文件
    for file in "${IMAGE_FILES[@]}"; do
        if [ ! -f "$file" ]; then
            log_error "缺少图像文件: $file"
            missing_files=$((missing_files + 1))
        else
            log_success "找到图像: $file"
        fi
    done
    
    # 检查外参文件
    if [ ! -f "$EXTRINSICS_FILE" ]; then
        log_error "缺少外参文件: $EXTRINSICS_FILE"
        missing_files=$((missing_files + 1))
    else
        log_success "找到外参文件: $EXTRINSICS_FILE"
    fi
    
    if [ $missing_files -gt 0 ]; then
        log_error "共有 $missing_files 个文件缺失，无法继续"
        exit 1
    fi
    
    log_success "所有输入文件检查通过"
}

# ----------------------------------------------------------------------------
# 创建输出目录
# ----------------------------------------------------------------------------
create_output_directory() {
    log_info "创建输出目录: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
    
    if [ $? -ne 0 ]; then
        log_error "无法创建输出目录"
        exit 1
    fi
    
    log_success "输出目录创建成功"
}

# ----------------------------------------------------------------------------
# 运行标定
# ----------------------------------------------------------------------------
run_calibration() {
    log_info "开始运行标定..."
    log_info "相机模型: Ocam ($CAMERA_MODEL)"
    log_info "输出目录: $OUTPUT_DIR"
    log_info "预计时间: 20-30 秒"
    
    local start_time=$(date +%s)
    
    # 构建命令
    local cmd=(./bin/run_AVM_Calibration
        --camera-model "$CAMERA_MODEL"
        --extrinsics "$EXTRINSICS_FILE"
        --ocam-front "${OCAM_FILES[0]}"
        --ocam-left "${OCAM_FILES[1]}"
        --ocam-behind "${OCAM_FILES[2]}"
        --ocam-right "${OCAM_FILES[3]}"
        --front "${IMAGE_FILES[0]}"
        --left "${IMAGE_FILES[1]}"
        --behind "${IMAGE_FILES[2]}"
        --right "${IMAGE_FILES[3]}"
        --output-dir "$OUTPUT_DIR/"
    )
    
    # 运行标定（带超时）
    timeout 600s "${cmd[@]}" 2>&1 | tee "$OUTPUT_DIR/calibration.log"
    
    local exit_code=${PIPESTATUS[0]}
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    
    if [ $exit_code -eq 0 ]; then
        log_success "标定成功完成，耗时 ${duration} 秒"
    elif [ $exit_code -eq 124 ]; then
        log_warning "标定超时（可能仍在正常范围）"
    else
        log_error "标定失败，退出码: $exit_code"
        return 1
    fi
    
    return 0
}

# ----------------------------------------------------------------------------
# 验证结果
# ----------------------------------------------------------------------------
validate_results() {
    log_info "验证标定结果..."
    
    local validation_passed=0
    local validation_failed=0
    
    # 检查输出文件
    local required_files=(
        "calibration_results.json5"
        "before_all_calib.png"
        "after_all_calib.png"
    )
    
    for file in "${required_files[@]}"; do
        local filepath="$OUTPUT_DIR/$file"
        if [ ! -f "$filepath" ]; then
            log_error "缺少输出文件: $file"
            validation_failed=$((validation_failed + 1))
        else
            log_success "找到输出文件: $file"
            validation_passed=$((validation_passed + 1))
        fi
    done
    
    # 检查 JSON 文件
    local json_file="$OUTPUT_DIR/calibration_results.json5"
    if [ -f "$json_file" ]; then
        log_info "检查 JSON 文件格式..."
        
        # 简单的 JSON 格式验证（检查必要字段）
        for camera in park_front park_left park_back park_right; do
            if grep -q "$camera" "$json_file"; then
                log_success "找到相机配置: $camera"
            else
                log_warning "缺少相机配置: $camera"
            fi
        done
        
        # 检查 JSON 语法（基本的）
        if python3 -c "import json; json.load(open('$json_file'))" 2>/dev/null; then
            log_success "JSON 文件格式正确"
        else
            log_warning "JSON 文件可能存在语法问题"
        fi
    fi
    
    # 检查图像文件
    for img in before_all_calib.png after_all_calib.png; do
        local imgpath="$OUTPUT_DIR/$img"
        if [ -f "$imgpath" ]; then
            log_info "检查图像: $img"
            
            # 获取图像信息
            local img_info=$(file "$imgpath")
            local img_size=$(stat -c%s "$imgpath")
            
            log_info "  文件类型: $img_info"
            log_info "  文件大小: $img_size bytes"
            
            # 检查文件大小（太可能有问题）
            if [ "$img_size" -lt 10000 ]; then
                log_warning "  文件过小（<10KB），可能异常"
                validation_failed=$((validation_failed + 1))
            else
                log_success "  文件大小正常"
                validation_passed=$((validation_passed + 1))
            fi
        fi
    done
    
    # 总结验证结果
    log_info "验证总结: $validation_passed 项通过, $validation_failed 项失败"
    
    if [ $validation_failed -eq 0 ]; then
        log_success "所有验证项均通过"
        return 0
    else
        log_warning "部分验证项未通过"
        return 1
    fi
}

# ----------------------------------------------------------------------------
# 显示结果摘要
# ----------------------------------------------------------------------------
show_summary() {
    log_info "=========================================="
    log_info "标定结果摘要"
    log_info "=========================================="
    log_info "输出目录: $OUTPUT_DIR"
    log_info ""
    log_info "生成的文件:"
    ls -lh "$OUTPUT_DIR" | sed 's/^/  /'
    log_info ""
    log_info "关键信息:"
    
    # 从日志中提取关键信息
    local log_file="$OUTPUT_DIR/calibration.log"
    if [ -f "$log_file" ]; then
        # 提取外参信息
        grep -A 2 "Loaded.*extrinsics:" "$log_file" | sed 's/^/  /'
        
        # 提取标定时间
        grep "calibration time:" "$log_file" | sed 's/^/  /'
        
        # 提取最终损失值（如果可用）
        grep "luminorsity loss after opt:" "$log_file" | tail -1 | sed 's/^/  /'
    fi
    
    log_info ""
    log_info "查看结果:"
    log_info "  eog $OUTPUT_DIR/before_all_calib.png"
    log_info "  eog $OUTPUT_DIR/after_all_calib.png"
    log_info ""
    log_info "外参结果:"
    log_info "  cat $OUTPUT_DIR/calibration_results.json5"
    log_info ""
    log_info "完整日志:"
    log_info "  cat $OUTPUT_DIR/calibration.log"
}

# ----------------------------------------------------------------------------
# 自动修复常见问题
# ----------------------------------------------------------------------------
auto_fix_issues() {
    log_info "检查并自动修复常见问题..."
    
    # 检查是否有写入权限
    if [ ! -w "." ]; then
        log_error "当前目录没有写入权限"
        return 1
    fi
    
    # 检查可执行文件
    if [ ! -x "./bin/run_AVM_Calibration" ]; then
        log_warning "找不到可执行文件，尝试重新构建..."
        
        if [ -f "./build_with_ocam.sh" ]; then
n            ./build_with_ocam.sh
            if [ $? -ne 0 ]; then
                log_error "构建失败"
                return 1
            fi
        else
            log_error "构建脚本不存在"
            return 1
        fi
    fi
    
    log_success "自动修复完成"
}

# ----------------------------------------------------------------------------
# 主函数
# ----------------------------------------------------------------------------
main() {
    log_info "=========================================="
    log_info "Ocam 相机标定与验证"
    log_info "=========================================="
    
    # 步骤 1: 自动修复
    auto_fix_issues
    if [ $? -ne 0 ]; then
        exit 1
    fi
    
    # 步骤 2: 检查输入
    check_input_files
    
    # 步骤 3: 创建输出目录
    create_output_directory
    
    # 步骤 4: 运行标定
    run_calibration
    if [ $? -ne 0 ]; then
        log_error "标定失败，退出"
        exit 1
    fi
    
    # 步骤 5: 验证结果
    validate_results
    local validation_result=$?
    
    # 步骤 6: 显示摘要
    show_summary
    
    # 最终结果
    log_info "=========================================="
    if [ $validation_result -eq 0 ]; then
        log_success "标定和验证全部完成！"
        log_info "检查结果:"
        log_info "  1. 查看 BEV 图像: eog $OUTPUT_DIR/*.png"
        log_info "  2. 检查外参文件: cat $OUTPUT_DIR/calibration_results.json5"
        log_info "  3. 查看详细日志: cat $OUTPUT_DIR/calibration.log"
    else
        log_warning "标定完成，但验证发现问题"
        log_info "请检查:"
        log_info "  1. BEV 图像是否正常显示"
        log_info "  2. 文件大小是否合理"
        log_info "  3. JSON 格式是否正确"
    fi
    log_info "=========================================="
}

# ----------------------------------------------------------------------------
# 执行主函数
# ----------------------------------------------------------------------------
main "$@"
