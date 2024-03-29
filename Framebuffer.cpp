/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

#include "Framebuffer.h"

Framebuffer::Framebuffer() {
    this->clear();
}

#ifndef SIMULATOR
void Framebuffer::drawBitmap(const uint8_t *progmem_bitmap, uint8_t height, uint8_t width, uint8_t pos_x, uint8_t pos_y) {
    uint8_t current_byte;
    uint8_t byte_width = (width + 7)/8;

    for (uint8_t current_y = 0; current_y < height; current_y++) {
        for (uint8_t current_x = 0; current_x < width; current_x++) {
            current_byte = pgm_read_byte(progmem_bitmap + current_y*byte_width + current_x/8);
            if (current_byte & (128 >> (current_x&7))) {
                this->drawPixel(current_x+pos_x,current_y+pos_y,1);
            } else {
                this->drawPixel(current_x+pos_x,current_y+pos_y,0);
            }
        }
    }
}


void Framebuffer::drawBuffer(const uint8_t *progmem_buffer) {
    uint8_t current_byte;

    for (uint8_t y_pos = 0; y_pos < 64; y_pos++) {
        for (uint8_t x_pos = 0; x_pos < 128; x_pos++) {
            current_byte = pgm_read_byte(progmem_buffer + y_pos*16 + x_pos/8);
            if (current_byte & (128 >> (x_pos&7))) {
                this->drawPixel(x_pos,y_pos,1);
            } else {
                this->drawPixel(x_pos,y_pos,0);
            }
        }
    }
}
#endif

void Framebuffer::drawPixel(uint8_t pos_x, uint8_t pos_y, uint8_t pixel_status) {
    if (pos_x >= SSD1306_WIDTH || pos_y >= SSD1306_HEIGHT) {
        return;
    }

    if (pixel_status) {
        this->buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] |= (1 << (pos_y&7));
    } else {
        this->buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] &= ~(1 << (pos_y&7));
    }
}

void Framebuffer::drawPixel(uint8_t pos_x, uint8_t pos_y) {
    if (pos_x >= SSD1306_WIDTH || pos_y >= SSD1306_HEIGHT) {
        return;
    }

    this->buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] |= (1 << (pos_y&7));
}

void Framebuffer::drawVLine(uint8_t x, uint8_t y, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) {
        this->drawPixel(x,i+y);
    }
}

void Framebuffer::drawHLine(uint8_t x, uint8_t y, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) {
        this->drawPixel(i+x,y);
    }
}

void Framebuffer::drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2){
    int dx = fabs(x2 - x1);
    int dy = fabs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;

    int err = dx - dy;

    while (true) {
        // 대각선을 그림
        this->drawPixel(x1,y1);
        if (x1 == x2 && y1 == y2) {
            break;
        }

        int e2 = 2 * err;

        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }

        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

void Framebuffer::rotateX3D(float theta){
    float sinTheta = sin(theta);
    float cosTheta = cos(theta);

    for (int n = 0; n < numNodes; n++) {
        float y = this->nodes[n][1];
        float z = this->nodes[n][2];

        this->nodes[n][1] = y * cosTheta - z * sinTheta;
        this->nodes[n][2] = z * cosTheta + y * sinTheta;
    }
}

void Framebuffer::rotateY3D(float theta){
    float sinTheta = sin(theta);
    float cosTheta = cos(theta);

    for (int n = 0; n < numNodes; n++) {
        float x = this->nodes[n][0];
        float z = this->nodes[n][2];

        this->nodes[n][0] = x * cosTheta - z * sinTheta;
        this->nodes[n][2] = z * cosTheta + x * sinTheta;
    }
}   

void Framebuffer::rotateZ3D(float theta){
    float sinTheta = sin(theta);
    float cosTheta = cos(theta);

    for (int n = 0; n < numNodes; n++) {
        float x = this->nodes[n][0];
        float y = this->nodes[n][1];

        this->nodes[n][0] = x * cosTheta - y * sinTheta;
        this->nodes[n][1] = y * cosTheta + x * sinTheta;
    }
}

void Framebuffer::setCubePosition(float x, float y, float z) {
        // Cube 위치 설정
    for (int n = 0; n < 8; ++n) {
        nodes[n][0] += x;
        nodes[n][1] += y;
        nodes[n][2] -= z;
    }
}

void Framebuffer::drawCube(){
    for (int i = 0; i < numEdges; i++)
    {
        int n0 = this->edges[i][0];
        int n1 = this->edges[i][1];
        float node0[3] = { this->nodes[n0][0], this->nodes[n0][1], this->nodes[n0][2] };
        float node1[3] = { this->nodes[n1][0], this->nodes[n1][1], this->nodes[n1][2] };
        this->drawLine(node0[0], node0[1], node1[0], node1[1]);
    }

}

void Framebuffer::Cube_default(){
    float cube[8][3] = {
        {-16.0f, -16.0f, -16.0f},
        {-16.0f, -16.0f, 16.0f},
        {-16.0f, 16.0f, -16.0f},
        {-16.0f, 16.0f, 16.0f},
        {16.0f, -16.0f, -16.0f},
        {16.0f, -16.0f, 16.0f},
        {16.0f, 16.0f, -16.0f},
        {16.0f, 16.0f, 16.0f}
    };

    // Copy the values to this->nodes[numNodes][3]
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 3; ++j) {
            this->nodes[i][j] = cube[i][j];
        }
    }
}

void Framebuffer::drawHexagon(int centerX, int centerY, int sideLength) {
    // 원의 내접 정육각형의 꼭지점 계산
    for (int i = 0; i < 6; ++i) {
        double angleRad = 2 * PI * i / 6;
        int x = (int)(centerX + sideLength * cos(angleRad));
        int y = (int)(centerY + sideLength * sin(angleRad));
        this->drawPixel(x,y);
    }
}

void Framebuffer::drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    uint8_t length = x2 - x1 + 1;
    uint8_t height = y2 - y1;

    this->drawHLine(x1,y1,length);
    this->drawHLine(x1,y2,length);
    this->drawVLine(x1,y1,height);
    this->drawVLine(x2,y1,height);
}

void Framebuffer::drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill) {
    if (!fill) {
        this->drawRectangle(x1,y1,x2,y2);
    } else {
        uint8_t length = x2 - x1 + 1;
        uint8_t height = y2 - y1;

        for (int x = 0; x < length; ++x) {
            for (int y = 0; y <= height; ++y) {
                this->drawPixel(x1+x,y+y1);
            }
        }
    }
}

void Framebuffer::clear() {
    for (uint16_t buffer_location = 0; buffer_location < SSD1306_BUFFERSIZE; buffer_location++) {
        this->buffer[buffer_location] = 0x00;
    }
}

void Framebuffer::invert(uint8_t status) {
    this->oled.invert(status);
}

void Framebuffer::show() {
    this->oled.sendFramebuffer(this->buffer);
}

void Framebuffer::displayChar(char character, int x, int y) {
  if (x >= 0 && x < 128 && y >= 0 && y < 64) {
    // Calculate the index in the font array
    int fontIndex = (character - 32) * 16 + 4;

    // Draw the character on the OLED screen
    for (int i = 0; i < 8; i++) {
      for (int j = 0; j < 8; j++) {
        if (pgm_read_byte(&ssd1306xled_font8x16[fontIndex + i]) & (1 << (j))) {
          this->drawPixel(x + i, y + j);
        }
      }
    }
    for (int i = 8; i < 16; i++) {
      for (int j = 0; j < 8; j++) {
        if (pgm_read_byte(&ssd1306xled_font8x16[fontIndex + i]) & (1 << (j))) {
          this->drawPixel(x + i-8, y + j+8);
        }
      }
    }
  }
}

void Framebuffer::printStringOnOLED(uint8_t pos_x, uint8_t pos_y, const char *str)
{
    
    UART_Printf("test\n");
    // 문자열의 각 문자에 대해 폰트를 이용하여 출력
    // while (*str)
    // {
    //     char c = *str;
    //     if (c >= 65 && c <= 126)
    //     {
    //         // ASCII 코드가 폰트 배열에 있는 경우에만 출력
    //         int index = c - 65; // ASCII 65부터 시작하므로 65를 빼서 인덱스 계산
    //         for (int i = 0; i < 8; i++)
    //         {
    //             for (int j = 0; j < 2; j++)
    //             {
    //                 if (pgm_read_byte(&(FONT8X15[index][i][j])) == 1)
    //                 {
    //                     // OLED에 픽셀 그리기
    //                     this->drawPixel(pos_x + i, pos_y + j);
    //                     UART_Printf("%d,%d",pos_x + i, pos_y + j);
    //                 }
    //             }
    //         }
    //         // 다음 문자의 출력 위치로 이동
    //         pos_x += 8;
    //     }
    //     str++;
    // }
for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            uint8_t byteValue = pgm_read_byte(&(FONT8X2[3][i][j]));  

            for (int k = 0; k < 8; k++)
            {
                // Check if the k-th bit is set in the byte
                if (byteValue & (1 << k))
                {
                    // OLED에 픽셀 그리기
                    this->drawPixel(pos_x + i, pos_y + j * 8 + k);
                }
            }
        }
    }
}