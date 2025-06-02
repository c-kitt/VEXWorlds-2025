#include "lib/lights.hpp"
#include "robotconfig.h"
#include <cmath>
#include <string>
#include <vector>



struct RGB {
    uint8_t r, g, b;
    RGB(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
};
struct HSV {
    double h; // Range: 0-360 (degrees)
    double s; // Range: 0-1 (percentage, normalized)
    double v; // Range: 0-1 (percentage, normalized)
    HSV(double h, double s, double v) : h(h), s(s), v(v) {}
};



HSV rgbToHsv(const RGB& rgb) {
    // Normalize RGB values to the range [0, 1]
    double r = rgb.r / 255.0;
    double g = rgb.g / 255.0;
    double b = rgb.b / 255.0;

    // Find the min and max values among r, g, and b
    double maxVal = std::max({r, g, b});
    double minVal = std::min({r, g, b});
    double delta = maxVal - minVal;

    HSV hsv(0, 0, 0);
    hsv.v = maxVal; // Value component

    // Compute saturation
    if (maxVal == 0) {
        hsv.s = 0; // If max is 0, saturation is 0 (black color)
    } else {
        hsv.s = delta / maxVal;
    }

    // Compute hue
    if (delta == 0) {
        hsv.h = 0; // If no difference, hue is undefined (set to 0)
    } else {
        if (maxVal == r) {
            hsv.h = 60.0 * (fmod(((g - b) / delta), 6));
        } else if (maxVal == g) {
            hsv.h = 60.0 * (((b - r) / delta) + 2);
        } else if (maxVal == b) {
            hsv.h = 60.0 * (((r - g) / delta) + 4);
        }

        if (hsv.h < 0) {
            hsv.h += 360.0; // Ensure hue is non-negative
        }
    }

    return hsv;
}



RGB hsvToRgb(const HSV& hsv) {
    RGB rgb(0, 0, 0);

    if (hsv.s == 0) {
        // If saturation is 0, the color is a shade of gray
        rgb.r = rgb.g = rgb.b = hsv.v * 255; // v is the brightness
    } else {
        double c = hsv.v * hsv.s; // Chroma
        double x = c * (1 - std::fabs(fmod(hsv.h / 60.0, 2) - 1));
        double m = hsv.v - c; // Match value

        double r, g, b;

        // Determine the RGB components based on the hue value
        if (hsv.h >= 0 && hsv.h < 60) {
            r = c; g = x; b = 0;
        } else if (hsv.h >= 60 && hsv.h < 120) {
            r = x; g = c; b = 0;
        } else if (hsv.h >= 120 && hsv.h < 180) {
            r = 0; g = c; b = x;
        } else if (hsv.h >= 180 && hsv.h < 240) {
            r = 0; g = x; b = c;
        } else if (hsv.h >= 240 && hsv.h < 300) {
            r = x; g = 0; b = c;
        } else { // hsv.h >= 300 && hsv.h < 360
            r = c; g = 0; b = x;
        }

        // Adjust the RGB values by adding m
        rgb.r = (r + m) * 255;
        rgb.g = (g + m) * 255;
        rgb.b = (b + m) * 255;
    }

    return rgb;
}



int wrapDegrees(float degrees) {
    while (degrees > 360) {
        degrees -= 360;
    }
    while (degrees < 0) {
        degrees += 360;
    }
    return degrees;
}



int hexToDecimal(const std::string& hex) {
    return std::stoi(hex, nullptr, 16);
}



RGB hexToRGB(const std::string& hex) {
    std::string cleanHex = hex;
    if (cleanHex[0] == '#') {
        cleanHex = cleanHex.substr(1);
    }
    
    int r = hexToDecimal(cleanHex.substr(0, 2));
    int g = hexToDecimal(cleanHex.substr(2, 2));
    int b = hexToDecimal(cleanHex.substr(4, 2));
    
    return RGB(r, g, b);
}



std::vector<RGB> interpolateColors(float start, float end, int stripLength) {
    HSV color1 = HSV(start, 1, 1);
    HSV color2 = HSV(end, 1, 1);
    
    std::vector<RGB> result;
    result.reserve(stripLength * 2);
    
    if (stripLength == 1) {
        result.push_back(hsvToRgb(color1));
        return result;
    }
    

    for (int i = 0; i < stripLength; ++i) {

        int h = std::lerp(static_cast<float>(color1.h), static_cast<float>(color2.h), static_cast<float>(i) / stripLength);
        int s = std::lerp(static_cast<float>(color1.s), static_cast<float>(color2.s), static_cast<float>(i) / stripLength);
        int v = std::lerp(static_cast<float>(color1.v), static_cast<float>(color2.v), static_cast<float>(i) / stripLength);
        
        result.emplace_back(hsvToRgb(HSV(wrapDegrees(h), s, v)));
    }
    for (int i = stripLength; i > 0; --i) {

        int h = std::lerp(static_cast<float>(color1.h), static_cast<float>(color2.h), static_cast<float>(i) / stripLength);
        int s = std::lerp(static_cast<float>(color1.s), static_cast<float>(color2.s), static_cast<float>(i) / stripLength);
        int v = std::lerp(static_cast<float>(color1.v), static_cast<float>(color2.v), static_cast<float>(i) / stripLength);
        
        result.emplace_back(hsvToRgb(HSV(wrapDegrees(h), s, v)));
    }
    
    return result;
}



void lib::Lights::loop() {
    team currentTeam = teamColor;
    int currentWarning = 0;
    int offset = 0;
    


    std::vector<RGB> blue = interpolateColors(120, 300, 50);
    std::vector<RGB> red = interpolateColors(325, 390, 50);
    std::vector<RGB> skills = red; //interpolateColors(260, 338, 40);

    std::vector<RGB> warning1 = interpolateColors(100, 180, 40);
    std::vector<RGB> warning2 = interpolateColors(50, 140, 40);



    // Calculate colors
    std::vector<uint32_t> stripColors(100);
    for (int i = 0; i < 100; i++) {
        RGB color = teamColor == team::red ? red[i] : teamColor == team::blue ? blue[i] : skills[i];
        stripColors[i] = (static_cast<uint32_t>(color.r) << 16) | 
                        (static_cast<uint32_t>(color.g) << 8) | 
                        static_cast<uint32_t>(color.b);
    }


    // Fill effect
    for (int i = 0; i < 40; i++) {
        int colorIndex = (i + offset) % 40;
        leftDriveLed.set_pixel(stripColors[colorIndex], i);
        pros::delay(10);

        rightDriveLed.set_pixel(stripColors[colorIndex], i);
        pros::delay(10);
    }
    offset = (offset + 1) % 40;
    


    while (true) {


        /*if (startTime != -1) {
            int dT = pros::millis() - startTime;

            // set to time warning 1
            if (5000 < dT && dT < 6500) {
                if (currentWarning != 1) {
                    currentWarning = 1;
                    for (int i = 0; i < 40; i++) {
                        RGB color = warning1[i];
                        stripColors[i] = (static_cast<uint32_t>(color.r) << 16) | 
                                        (static_cast<uint32_t>(color.g) << 8) | 
                                        static_cast<uint32_t>(color.b);
                    }
                }
            }

            // set to time warning 2
            else if (10000 < dT && dT < 11500) {
                if (currentWarning != 2) {
                    currentWarning = 2;
                    for (int i = 0; i < 40; i++) {
                        RGB color = warning2[i];
                        stripColors[i] = (static_cast<uint32_t>(color.r) << 16) | 
                                        (static_cast<uint32_t>(color.g) << 8) | 
                                        static_cast<uint32_t>(color.b);
                    }
                }
            }
            else if (currentWarning != 0) {
                currentWarning = 0;
                currentTeam = -1;
            }
        }*/

        // set to team colors
        if (teamColor != currentTeam && currentWarning == 0) {
            currentTeam = teamColor;
            for (int i = 0; i < 100; i++) {
                RGB color = teamColor == team::red ? red[i] : teamColor == team::blue ? blue[i] : skills[i];
                stripColors[i] = (static_cast<uint32_t>(color.r) << 16) | 
                                (static_cast<uint32_t>(color.g) << 8) | 
                                static_cast<uint32_t>(color.b);
            }
        }

        // Update left strip
        for (int i = 0; i < 40; i++) {
            int colorIndex = (i + offset) % 100;
            leftDriveLed.set_pixel(stripColors[colorIndex], i);
        }
        pros::delay(10);
            
        
        // Update right strip
        for (int i = 0; i < 40; i++) {
            int colorIndex = (i + offset) % 100;
            rightDriveLed.set_pixel(stripColors[colorIndex], i);
        }
        pros::delay(10);


        // update indicator
        if (clamp.is_extended()) {
            for (int i = 0; i < 40; i++) {
                int colorIndex = (i + offset) % 100;
                indicatorLed1.set_pixel(stripColors[colorIndex], i);
            }
            pros::delay(10);

        } else {
            indicatorLed1.clear();
            pros::delay(10);
        }


        offset = (offset + 1) % 100;
    }
}