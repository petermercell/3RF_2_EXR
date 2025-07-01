// batch_3fr_to_exr.cpp
#include <libraw/libraw.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfRgba.h>
#include <OpenEXR/ImfArray.h>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#ifdef _WIN32
#include <direct.h>
#define mkdir _mkdir
#endif

using namespace Imf;
using namespace Imath;

bool convert3frToExr(const std::string& inputPath, const std::string& outputPath) {
    LibRaw processor;
    
    // Open the 3FR file
    int ret = processor.open_file(inputPath.c_str());
    if (ret != LIBRAW_SUCCESS) {
        std::cout << "Failed to open " << inputPath << ": " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    std::cout << "Processing: " << inputPath << std::endl;
    
    // Print sensor information
    int full_width = processor.imgdata.sizes.raw_width;
    int full_height = processor.imgdata.sizes.raw_height;
    int visible_width = processor.imgdata.sizes.width;
    int visible_height = processor.imgdata.sizes.height;
    int top_margin = processor.imgdata.sizes.top_margin;
    int left_margin = processor.imgdata.sizes.left_margin;
    
    std::cout << "Raw size: " << full_width << "x" << full_height << std::endl;
    std::cout << "Visible size: " << visible_width << "x" << visible_height << std::endl;
    std::cout << "Top margin: " << top_margin << std::endl;
    std::cout << "Left margin: " << left_margin << std::endl;
    
    // Configure LibRaw processing parameters
    processor.imgdata.params.use_camera_wb = 1;      // Use camera white balance
    processor.imgdata.params.output_color = 1;       // sRGB color space
    processor.imgdata.params.output_bps = 16;        // 16-bit output
    processor.imgdata.params.no_auto_bright = 1;     // Disable auto brightness
    processor.imgdata.params.gamm[0] = 1.0;         // Linear gamma (no gamma correction)
    processor.imgdata.params.gamm[1] = 1.0;
    processor.imgdata.params.bright = 1.0;          // No brightness adjustment
    processor.imgdata.params.user_qual = 3;         // High quality demosaicing (AHD)
    
    // Temporarily store original crop settings
    int orig_width = processor.imgdata.sizes.width;
    int orig_height = processor.imgdata.sizes.height;
    int orig_top = processor.imgdata.sizes.top_margin;
    int orig_left = processor.imgdata.sizes.left_margin;
    
    // Modify sizes to use full sensor
    processor.imgdata.sizes.width = full_width;
    processor.imgdata.sizes.height = full_height;
    processor.imgdata.sizes.top_margin = 0;
    processor.imgdata.sizes.left_margin = 0;
    processor.imgdata.sizes.iwidth = full_width;
    processor.imgdata.sizes.iheight = full_height;
    
    // Unpack the RAW data
    ret = processor.unpack();
    if (ret != LIBRAW_SUCCESS) {
        std::cout << "Failed to unpack: " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    std::cout << "Unpacked successfully!" << std::endl;
    
    // Process the image using LibRaw's pipeline
    ret = processor.dcraw_process();
    if (ret != LIBRAW_SUCCESS) {
        std::cout << "Failed to process: " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    std::cout << "Processed successfully!" << std::endl;
    
    // Get the processed image
    libraw_processed_image_t *processed_image = processor.dcraw_make_mem_image(&ret);
    if (!processed_image) {
        std::cout << "Failed to create processed image: " << libraw_strerror(ret) << std::endl;
        return false;
    }
    
    std::cout << "Created processed image!" << std::endl;
    std::cout << "Processed image size: " << processed_image->width << "x" << processed_image->height << std::endl;
    std::cout << "Colors: " << processed_image->colors << ", bits: " << processed_image->bits << std::endl;
    
    try {
        // Create EXR file
        RgbaOutputFile file(outputPath.c_str(), 
                           processed_image->width, processed_image->height, WRITE_RGBA);
        
        // Create pixel array
        Array2D<Rgba> pixels(processed_image->height, processed_image->width);
        
        // Convert processed image data to EXR format
        if (processed_image->colors == 3 && processed_image->bits == 16) {
            // RGB 16-bit data
            unsigned short *img_data = (unsigned short*)processed_image->data;
            
            for (int y = 0; y < processed_image->height; ++y) {
                for (int x = 0; x < processed_image->width; ++x) {
                    int idx = (y * processed_image->width + x) * 3;
                    
                    // Convert from 16-bit to float (0-1 range)
                    pixels[y][x].r = img_data[idx + 0] / 65535.0f;
                    pixels[y][x].g = img_data[idx + 1] / 65535.0f;
                    pixels[y][x].b = img_data[idx + 2] / 65535.0f;
                    pixels[y][x].a = 1.0f;
                }
            }
        } else if (processed_image->colors == 3 && processed_image->bits == 8) {
            // RGB 8-bit data
            unsigned char *img_data = processed_image->data;
            
            for (int y = 0; y < processed_image->height; ++y) {
                for (int x = 0; x < processed_image->width; ++x) {
                    int idx = (y * processed_image->width + x) * 3;
                    
                    // Convert from 8-bit to float (0-1 range)
                    pixels[y][x].r = img_data[idx + 0] / 255.0f;
                    pixels[y][x].g = img_data[idx + 1] / 255.0f;
                    pixels[y][x].b = img_data[idx + 2] / 255.0f;
                    pixels[y][x].a = 1.0f;
                }
            }
        } else {
            std::cout << "Unsupported image format: " << processed_image->colors 
                      << " colors, " << processed_image->bits << " bits" << std::endl;
            LibRaw::dcraw_clear_mem(processed_image);
            return false;
        }
        
        // Write the pixels to the EXR file
        file.setFrameBuffer(&pixels[0][0], 1, processed_image->width);
        file.writePixels(processed_image->height);
        
        std::cout << "Full sensor EXR file saved successfully to " << outputPath << std::endl;
        std::cout << "Final size: " << processed_image->width << "x" << processed_image->height << std::endl;
        
    } catch (const std::exception &e) {
        std::cout << "EXR write error: " << e.what() << std::endl;
        LibRaw::dcraw_clear_mem(processed_image);
        return false;
    }
    
    // Clean up
    LibRaw::dcraw_clear_mem(processed_image);
    
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
    
    // Check command line arguments
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <input_directory>" << std::endl;
        std::cout << "Example: " << argv[0] << " /path/to/3fr/files" << std::endl;
        return 1;
    }
    
    inputDir = argv[1];
    
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
        
        if (convert3frToExr(inputFile, outputFile)) {
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