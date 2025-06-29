// batch_3fr_to_exr.cpp
#include <libraw/libraw.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfRgba.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <Imath/half.h>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#ifdef _WIN32
#include <direct.h>
#define mkdir _mkdir
#endif

using namespace Imf;
using namespace Imath;

// sRGB gamma correction function
float applySRGBCurve(float linear) {
    // Clamp input to valid range
    linear = std::max(0.0f, std::min(1.0f, linear));
    
    // Apply sRGB gamma curve
    if (linear <= 0.0031308f) {
        return 12.92f * linear;
    } else {
        return 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
    }
}

// Optional: Inverse sRGB curve (if you want to convert sRGB back to linear)
float applySRGBInverse(float srgb) {
    srgb = std::max(0.0f, std::min(1.0f, srgb));
    
    if (srgb <= 0.04045f) {
        return srgb / 12.92f;
    } else {
        return std::pow((srgb + 0.055f) / 1.055f, 2.4f);
    }
}

// Simple tone mapping function to prevent clipping
float simpleToneMap(float linear, float exposure = 1.0f) {
    // Apply exposure adjustment
    linear *= exposure;
    
    // Simple Reinhard tone mapping: x / (1 + x)
    return linear / (1.0f + linear);
}

bool convert3frToExr(const std::string& inputPath, const std::string& outputPath, bool useSRGB = true, float exposure = 1.0f) {
    LibRaw processor;
    
    // Open the 3FR file
    int ret = processor.open_file(inputPath.c_str());
    if (ret != LIBRAW_SUCCESS) {
        std::cout << "Failed to open " << inputPath << ": " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    std::cout << "Processing: " << inputPath << std::endl;
    std::cout << "Image size: " << processor.imgdata.sizes.width 
              << "x" << processor.imgdata.sizes.height << std::endl;
    
    // Unpack the RAW data
    ret = processor.unpack();
    if (ret != LIBRAW_SUCCESS) {
        std::cout << "Failed to unpack: " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    // Get raw sensor dimensions (full sensor including borders)
    int raw_width = processor.imgdata.sizes.raw_width;
    int raw_height = processor.imgdata.sizes.raw_height;
    int width = processor.imgdata.sizes.width;   // Visible area width
    int height = processor.imgdata.sizes.height; // Visible area height
    
    std::cout << "Raw sensor size: " << raw_width << "x" << raw_height << std::endl;
    std::cout << "Visible area: " << width << "x" << height << std::endl;
    
    // Force LibRaw to use the absolute full sensor area
    processor.imgdata.sizes.width = raw_width;
    processor.imgdata.sizes.height = raw_height;
    processor.imgdata.sizes.left_margin = 0;
    processor.imgdata.sizes.top_margin = 0;
    processor.imgdata.sizes.iwidth = raw_width;
    processor.imgdata.sizes.iheight = raw_height;
    processor.imgdata.sizes.raw_width = raw_width;
    processor.imgdata.sizes.raw_height = raw_height;
    
    // Disable all cropping in processing parameters
    processor.imgdata.params.use_auto_wb = 0;
    processor.imgdata.params.use_camera_wb = 1;
    processor.imgdata.params.no_auto_bright = 1;
    processor.imgdata.params.output_color = 1; // sRGB
    processor.imgdata.params.output_bps = 16;
    processor.imgdata.params.user_flip = 0; // No rotation
    processor.imgdata.params.user_qual = 3; // High quality demosaicing
    processor.imgdata.params.four_color_rgb = 0;
    processor.imgdata.params.highlight = 0; // No highlight recovery
    processor.imgdata.params.use_fuji_rotate = 0; // No Fuji rotation
    
    // Set output parameters for 16-bit
    processor.imgdata.params.output_bps = 16; // 16-bit output
    processor.imgdata.params.no_auto_bright = 1; // Preserve original exposure
    processor.imgdata.params.use_camera_wb = 1; // Use camera white balance
    
    // Process the image (demosaic, white balance, etc.) with full sensor area
    ret = processor.dcraw_process();
    if (ret != LIBRAW_SUCCESS) {
        std::cout << "Failed to process: " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    // Get processed image data
    libraw_processed_image_t *image = processor.dcraw_make_mem_image(&ret);
    if (!image) {
        std::cout << "Failed to make memory image: " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    int final_width = image->width;
    int final_height = image->height;
    int colors = image->colors; // Should be 3 for RGB
    
    std::cout << "Memory image created: " << final_width << "x" << final_height 
              << " with " << colors << " colors, " << image->bits << "-bit" << std::endl;
    std::cout << "Applying " << (useSRGB ? "inverse sRGB curve (sRGB->Linear)" : "linear") << " with exposure " << exposure << std::endl;
    
    try {
        // Create EXR file with full sensor processed RGB data
        RgbaOutputFile file(outputPath.c_str(), final_width, final_height, WRITE_RGBA);
        
        // Create pixel buffer using Rgba array
        Array2D<Rgba> pixels(final_height, final_width);
        
        // Convert LibRaw data to EXR format
        // Check if we have 16-bit data
        if (image->bits == 16) {
            unsigned short *data = (unsigned short*)image->data;
            
            for (int y = 0; y < final_height; ++y) {
                for (int x = 0; x < final_width; ++x) {
                    int idx = (y * final_width + x) * colors;
                    
                    // Convert from 16-bit to float and normalize to [0,1]
                    if (colors >= 3) {
                        float r = data[idx] / 65535.0f;
                        float g = data[idx + 1] / 65535.0f;
                        float b = data[idx + 2] / 65535.0f;
                        
                        // Apply inverse sRGB curve to convert from sRGB to linear
                        if (useSRGB) {
                            r = applySRGBInverse(r);
                            g = applySRGBInverse(g);
                            b = applySRGBInverse(b);
                        }
                        
                        // Apply tone mapping if exposure is not 1.0
                        if (exposure != 1.0f) {
                            r = simpleToneMap(r, exposure);
                            g = simpleToneMap(g, exposure);
                            b = simpleToneMap(b, exposure);
                        }
                        
                        pixels[y][x].r = r;
                        pixels[y][x].g = g;
                        pixels[y][x].b = b;
                        pixels[y][x].a = 1.0f; // Full alpha
                    } else {
                        // Grayscale
                        float gray = data[idx] / 65535.0f;
                        
                        // Apply inverse sRGB curve to convert from sRGB to linear
                        if (useSRGB) {
                            gray = applySRGBInverse(gray);
                        }
                        
                        // Apply tone mapping if exposure is not 1.0
                        if (exposure != 1.0f) {
                            gray = simpleToneMap(gray, exposure);
                        }
                        
                        pixels[y][x].r = gray;
                        pixels[y][x].g = gray;
                        pixels[y][x].b = gray;
                        pixels[y][x].a = 1.0f;
                    }
                }
            }
        } else {
            // Fallback for 8-bit data
            unsigned char *data = (unsigned char*)image->data;
            
            for (int y = 0; y < final_height; ++y) {
                for (int x = 0; x < final_width; ++x) {
                    int idx = (y * final_width + x) * colors;
                    
                    // Convert from 8-bit to float and normalize to [0,1]
                    if (colors >= 3) {
                        float r = data[idx] / 255.0f;
                        float g = data[idx + 1] / 255.0f;
                        float b = data[idx + 2] / 255.0f;
                        
                        // Apply inverse sRGB curve to convert from sRGB to linear
                        if (useSRGB) {
                            r = applySRGBInverse(r);
                            g = applySRGBInverse(g);
                            b = applySRGBInverse(b);
                        }
                        
                        // Apply tone mapping if exposure is not 1.0
                        if (exposure != 1.0f) {
                            r = simpleToneMap(r, exposure);
                            g = simpleToneMap(g, exposure);
                            b = simpleToneMap(b, exposure);
                        }
                        
                        pixels[y][x].r = r;
                        pixels[y][x].g = g;
                        pixels[y][x].b = b;
                        pixels[y][x].a = 1.0f; // Full alpha
                    } else {
                        // Grayscale
                        float gray = data[idx] / 255.0f;
                        
                        // Apply inverse sRGB curve to convert from sRGB to linear
                        if (useSRGB) {
                            gray = applySRGBInverse(gray);
                        }
                        
                        // Apply tone mapping if exposure is not 1.0
                        if (exposure != 1.0f) {
                            gray = simpleToneMap(gray, exposure);
                        }
                        
                        pixels[y][x].r = gray;
                        pixels[y][x].g = gray;
                        pixels[y][x].b = gray;
                        pixels[y][x].a = 1.0f;
                    }
                }
            }
        }
        
        // Write the pixels to the EXR file
        file.setFrameBuffer(&pixels[0][0], 1, final_width);
        file.writePixels(final_height);
        
        std::cout << "EXR file saved successfully to " << outputPath << std::endl;
        
    } catch (const std::exception &e) {
        std::cout << "EXR write error: " << e.what() << std::endl;
        LibRaw::dcraw_clear_mem(image);
        return false;
    }
    
    // Clean up
    LibRaw::dcraw_clear_mem(image);
    
    return true;
}

// Helper function to check if a file has .3fr extension (case insensitive)
bool is3frFile(const std::string& filename) {
    if (filename.length() < 4) return false;
    
    std::string extension = filename.substr(filename.length() - 4);
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    return extension == ".3fr";
}

// Helper function to get filename without extension
std::string getBasename(const std::string& filepath) {
    size_t lastSlash = filepath.find_last_of("/\\");
    size_t lastDot = filepath.find_last_of('.');
    
    size_t start = (lastSlash == std::string::npos) ? 0 : lastSlash + 1;
    size_t end = (lastDot == std::string::npos || lastDot < start) ? filepath.length() : lastDot;
    
    return filepath.substr(start, end - start);
}

// Helper function to check if directory exists
bool directoryExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        return false; // Cannot access path
    }
    return (info.st_mode & S_IFDIR) != 0;
}

// Helper function to create directory
bool createDirectory(const std::string& path) {
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

int main(int argc, char* argv[]) {
    std::string inputDir;
    bool useSRGB = true;
    float exposure = 1.0f;
    
    // Check command line arguments
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <input_directory> [--linear] [--exposure <value>]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  --linear     Keep sRGB output from LibRaw (no conversion to linear)" << std::endl;
        std::cout << "  --exposure   Set exposure multiplier (default: 1.0)" << std::endl;
        std::cout << "Example: " << argv[0] << " /path/to/3fr/files" << std::endl;
        std::cout << "Example: " << argv[0] << " /path/to/3fr/files --exposure 1.5" << std::endl;
        std::cout << "Example: " << argv[0] << " /path/to/3fr/files --linear" << std::endl;
        return 1;
    }
    
    inputDir = argv[1];
    
    // Parse additional arguments
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--linear") {
            useSRGB = false;
        } else if (arg == "--exposure") {
            if (i + 1 < argc) {
                exposure = std::stof(argv[++i]);
            } else {
                std::cout << "Error: --exposure requires a value" << std::endl;
                return 1;
            }
        }
    }
    
    // Ensure input directory path ends with /
    if (inputDir.back() != '/' && inputDir.back() != '\\') {
        inputDir += "/";
    }
    
    // Check if input directory exists
    if (!directoryExists(inputDir)) {
        std::cout << "Error: Input directory '" << inputDir << "' does not exist or is not a directory." << std::endl;
        return 1;
    }
    
    // Create output directory
    std::string outputDir = inputDir + "EXR";
    
    if (!directoryExists(outputDir)) {
        if (!createDirectory(outputDir)) {
            std::cout << "Error: Could not create output directory '" << outputDir << "'." << std::endl;
            return 1;
        }
        std::cout << "Created output directory: " << outputDir << std::endl;
    }
    
    // Add trailing slash to output directory
    if (outputDir.back() != '/' && outputDir.back() != '\\') {
        outputDir += "/";
    }
    
    // Find all 3FR files in the input directory
    std::vector<std::string> threeFrFiles;
    
    DIR* dir = opendir(inputDir.c_str());
    if (dir == nullptr) {
        std::cout << "Error: Could not open directory '" << inputDir << "'." << std::endl;
        return 1;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG || entry->d_type == DT_UNKNOWN) { // Regular file
            std::string filename(entry->d_name);
            if (is3frFile(filename)) {
                threeFrFiles.push_back(inputDir + filename);
            }
        }
    }
    closedir(dir);
    
    if (threeFrFiles.empty()) {
        std::cout << "No 3FR files found in directory: " << inputDir << std::endl;
        return 0;
    }
    
    std::cout << "Found " << threeFrFiles.size() << " 3FR file(s) to process:" << std::endl;
    for (const auto& file : threeFrFiles) {
        size_t lastSlash = file.find_last_of("/\\");
        std::string filename = (lastSlash == std::string::npos) ? file : file.substr(lastSlash + 1);
        std::cout << "  " << filename << std::endl;
    }
    std::cout << std::endl;
    
    std::cout << "Processing mode: " << (useSRGB ? "sRGB->Linear conversion" : "Linear (no conversion)") << std::endl;
    std::cout << "Exposure multiplier: " << exposure << std::endl;
    std::cout << std::endl;
    
    // Process each 3FR file
    int successCount = 0;
    int failCount = 0;
    
    for (const auto& inputFile : threeFrFiles) {
        // Get just the filename for display
        size_t lastSlash = inputFile.find_last_of("/\\");
        std::string inputFilename = (lastSlash == std::string::npos) ? inputFile : inputFile.substr(lastSlash + 1);
        
        // Create output filename by changing extension to .exr
        std::string basename = getBasename(inputFile);
        std::string outputFile = outputDir + basename + ".exr";
        
        std::cout << "Converting: " << inputFilename << " -> " << basename << ".exr" << std::endl;
        
        if (convert3frToExr(inputFile, outputFile, useSRGB, exposure)) {
            successCount++;
            std::cout << "✓ Successfully converted " << inputFilename << std::endl;
        } else {
            failCount++;
            std::cout << "✗ Failed to convert " << inputFilename << std::endl;
        }
        std::cout << "----------------------------------------" << std::endl;
    }
    
    // Summary
    std::cout << std::endl << "Batch conversion completed!" << std::endl;
    std::cout << "Successfully converted: " << successCount << " files" << std::endl;
    std::cout << "Failed conversions: " << failCount << " files" << std::endl;
    std::cout << "Output directory: " << outputDir << std::endl;
    
    return (failCount > 0) ? 1 : 0;
}