#!/usr/bin/env python3
"""
生成圆形图案的C数组，用于M5GFX显示
"""

from PIL import Image, ImageDraw
import os

def rgb888_to_rgb565(r, g, b):
    """将RGB888转换为RGB565格式"""
    r5 = (r >> 3) & 0x1F
    g6 = (g >> 2) & 0x3F
    b5 = (b >> 3) & 0x1F
    rgb565 = (r5 << 11) | (g6 << 5) | b5
    return rgb565

def image_to_rgb565_array(image, array_name):
    """将PIL Image转换为RGB565 C数组"""
    width, height = image.size
    pixels = image.load()
    
    # 生成数组声明
    output = f"// {width}x{height} RGB565格式\n"
    output += f"static const uint16_t {array_name}[{width * height}] = {{\n"
    
    # 转换每个像素
    for y in range(height):
        line = "    "
        for x in range(width):
            r, g, b = pixels[x, y][:3]  # 取RGB，忽略Alpha
            rgb565 = rgb888_to_rgb565(r, g, b)
            line += f"0x{rgb565:04X}, "
            
            # 每16个数据换行
            if (x + 1) % 16 == 0 and x < width - 1:
                output += line + "\n"
                line = "    "
        
        if line.strip() != "":
            output += line + "\n"
    
    output = output.rstrip(", \n") + "\n};\n"
    return output

def create_white_black_circle():
    """创建白底黑圆图案"""
    size = 466
    center_x = 233
    center_y = 233
    radius = 231
    
    # 创建白色背景图像
    img = Image.new('RGB', (size, size), (255, 255, 255))
    draw = ImageDraw.Draw(img)
    
    # 画黑色圆，圆心在(233, 233)
    left = center_x - radius
    top = center_y - radius
    right = center_x + radius
    bottom = center_y + radius
    draw.ellipse([left, top, right, bottom], fill=(0, 0, 0))
    
    return img

def create_red_white_circle():
    """创建红底白圆图案"""
    size = 466
    center_x = 233
    center_y = 233
    radius = 231
    
    # 创建红色背景图像
    img = Image.new('RGB', (size, size), (255, 0, 0))
    draw = ImageDraw.Draw(img)
    
    # 画白色圆，圆心在(233, 233)
    left = center_x - radius
    top = center_y - radius
    right = center_x + radius
    bottom = center_y + radius
    draw.ellipse([left, top, right, bottom], fill=(255, 255, 255))
    
    return img

def main():
    print("生成圆形图案...")
    
    # 创建输出目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'output')
    os.makedirs(output_dir, exist_ok=True)
    
    # 生成白底黑圆
    print("生成白底黑圆...")
    img1 = create_white_black_circle()
    img1.save(os.path.join(output_dir, 'white_black_circle.png'))
    array1 = image_to_rgb565_array(img1, "circle_white_black")
    
    # 生成红底白圆
    print("生成红底白圆...")
    img2 = create_red_white_circle()
    img2.save(os.path.join(output_dir, 'red_white_circle.png'))
    array2 = image_to_rgb565_array(img2, "circle_red_white")
    
    # 保存C头文件
    header_file = os.path.join(output_dir, 'circle_images.h')
    with open(header_file, 'w', encoding='utf-8') as f:
        f.write("/**\n")
        f.write(" * 圆形图案数据\n")
        f.write(" * 用于M5GFX显示\n")
        f.write(" * 使用方法: gfx.pushImage(x, y, 466, 466, circle_white_black);\n")
        f.write(" */\n\n")
        f.write("#ifndef CIRCLE_IMAGES_H\n")
        f.write("#define CIRCLE_IMAGES_H\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(array1)
        f.write("\n")
        f.write(array2)
        f.write("\n#endif // CIRCLE_IMAGES_H\n")
    
    print(f"完成！文件已保存到: {output_dir}")
    print(f"  - white_black_circle.png")
    print(f"  - red_white_circle.png")
    print(f"  - circle_images.h")
    print(f"\n使用方法:")
    print(f"  #include \"circle_images.h\"")
    print(f"  gfx.pushImage(0, 0, 466, 466, circle_white_black);")

if __name__ == '__main__':
    main()

