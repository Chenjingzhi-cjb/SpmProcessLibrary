#ifndef SPM_READER_HPP
#define SPM_READER_HPP

#include <iostream>
#include <fstream>
#include <regex>
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>


// example
int spmReaderExample0();


class SpmBase {
public:
    SpmBase() = default;

    ~SpmBase() = default;

protected:
    static int getIntFromTextByRegex(const std::string &regex_string, std::string &spm_file_text) {
        std::regex pattern(regex_string);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 2) {
            return std::stoi(matches[1].str());
        } else {
            return 0;
        }
    }

    static double getDoubleFromTextByRegex(const std::string &regex_string, std::string &spm_file_text) {
        std::regex pattern(regex_string);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 2) {
            return std::stod(matches[1].str());
        } else {
            return 0;
        }
    }

    static std::string getStringFromTextByRegex(const std::string &regex_string, std::string &spm_file_text) {
        std::regex pattern(regex_string);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 2) {
            return matches[1].str();
        } else {
            return "";
        }
    }
};


class SpmImage : public SpmBase {
public:
    SpmImage() = default;

    ~SpmImage() = default;

public:
    void parseImageAttributes(std::string &spm_file_text) {
        // Can be modified: 可添加需要的属性
        m_data_length = getIntFromTextByRegex(data_length_regex, spm_file_text);
        m_data_offset = getIntFromTextByRegex(data_offset_regex, spm_file_text);
        m_bytes_per_pixel = getIntFromTextByRegex(bytes_per_pixel_regex, spm_file_text);
        m_frame_direction = getStringFromTextByRegex(frame_direction_regex, spm_file_text);
        m_capture_start_line = getIntFromTextByRegex(capture_start_line_regex, spm_file_text);
        m_color_table_index = getIntFromTextByRegex(color_table_index_regex, spm_file_text);
        m_relative_frame_time = getDoubleFromTextByRegex(relative_frame_time_regex, spm_file_text);
        m_samps_per_line = getIntFromTextByRegex(samps_per_line_regex, spm_file_text);
        m_number_of_lines = getIntFromTextByRegex(number_of_lines_regex, spm_file_text);
    }

    void setZScale(std::string &spm_file_text, const std::string &z_scale_sens_str, double z_scale_sens) {
        m_z_scale = getZScaleFromTextByRegex(spm_file_text, z_scale_sens_str);
        m_z_scale_sens = z_scale_sens;
    }

    bool setImageData(std::vector<char> &byte_data) {
        // set raw data
        if (m_bytes_per_pixel == 2) {
            std::vector<short> raw_data_16;
            raw_data_16.assign(reinterpret_cast<const short *>(byte_data.data()),
                               reinterpret_cast<const short *>(byte_data.data() + byte_data.size()));
            m_raw_data.assign(raw_data_16.begin(), raw_data_16.end());
        } else if (m_bytes_per_pixel == 4) {
            m_raw_data.assign(reinterpret_cast<const int *>(byte_data.data()),
                              reinterpret_cast<const int *>(byte_data.data() + byte_data.size()));
        } else {
            return false;
        }

        // calc real data
        for (int r = (int) (m_number_of_lines - 1); r >= 0; r--) {
            std::vector<double> line_data;
            line_data.reserve(m_samps_per_line);
            for (int c = 0; c < m_samps_per_line; c++) {
                line_data.emplace_back(
                        m_raw_data[r * m_samps_per_line + c] * m_z_scale_sens * m_z_scale /
                        std::pow(2, 8 * m_bytes_per_pixel));
            }
            m_real_data.emplace_back(line_data);
        }

        return true;
    }

    unsigned int getDataLength() const { return m_data_length; }

    unsigned int getDataOffset() const { return m_data_offset; }

    std::vector<int> &getRawData() { return m_raw_data; }

    std::vector<std::vector<double>> &getRealData() { return m_real_data; }

    int getRows() const { return (int) m_number_of_lines; }

    int getCols() const { return (int) m_samps_per_line; }

private:
    // unit: V or mV
    // Since the units of sens are all nm/V, the units here are uniformly converted to V.
    static double getZScaleFromTextByRegex(std::string &spm_file_text, const std::string &z_scale_sens_str) {
        std::string z_scale_regex = R"(\@2:Z scale: V \[)" + z_scale_sens_str + R"(\] \(.*?\) (\d+\.\d+) ([mV]+))";
        std::regex pattern(z_scale_regex);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 3) {
            double value = std::stod(matches[1].str());
            std::string unit = matches[2].str();
            if (unit == "mV") value /= 1000;  // uniformly converted to V

            return value;
        } else {
            return 0;
        }
    }

public:
    // TODO: 1. 完善 Image Type
    enum class ImageType {
        HeightSensor,
        AmplitudeError,
        All
    };
    static const std::vector<std::string> image_type_str;

    // Can be modified: 可添加需要的属性正则表达式
    const std::string data_length_regex = R"(\Data length: (\d+))";
    const std::string data_offset_regex = R"(\Data offset: (\d+))";
    const std::string bytes_per_pixel_regex = R"(\Bytes/pixel: ([24]))";
    const std::string frame_direction_regex = R"(\Frame direction: ([A-Za-z]+))";
    const std::string capture_start_line_regex = R"(\Capture start line: (\d+))";
    const std::string color_table_index_regex = R"(\Color Table Index: (\d+))";
    const std::string relative_frame_time_regex = R"(\Relative frame time: (\d+(\.\d+)?))";
    const std::string samps_per_line_regex = R"(\Samps/line: (\d+))";
    const std::string number_of_lines_regex = R"(\Number of lines: (\d+))";

private:
    // Image attributes
    // Can be modified: 可添加需要的属性
    unsigned int m_data_length{};
    unsigned int m_data_offset{};
    unsigned int m_bytes_per_pixel{};
    std::string m_frame_direction;
    unsigned int m_capture_start_line{};
    unsigned int m_color_table_index{};
    double m_relative_frame_time{};
    unsigned int m_samps_per_line{};
    unsigned int m_number_of_lines{};

    double m_z_scale{};
    double m_z_scale_sens{};

    // Image data
    std::vector<int> m_raw_data;  // uniformly converted to 4 bytes (int)
    std::vector<std::vector<double>> m_real_data;
};

// TODO: 1. 完善 Image Type
const std::vector<std::string> SpmImage::image_type_str = std::vector<std::string>{
        "Height Sensor", "Amplitude Error"
};


class SpmReader : public SpmBase {
public:
    SpmReader(std::string spm_path, const std::string &image_type)
            : m_spm_path(std::move(spm_path)) {
        m_image_type_list.emplace_back(image_type);
    }

    explicit SpmReader(std::string spm_path, const SpmImage::ImageType &image_type = SpmImage::ImageType::HeightSensor)
            : m_spm_path(std::move(spm_path)) {
        if (image_type == SpmImage::ImageType::All) {
            m_image_type_list.assign(SpmImage::image_type_str.begin(), SpmImage::image_type_str.end());
        } else if ((int) image_type >= 0 && (int) image_type < SpmImage::image_type_str.size()) {
            m_image_type_list.emplace_back(SpmImage::image_type_str[(int) image_type]);
        }
    }

    SpmReader(std::string spm_path, std::vector<std::string> image_type_list)
            : m_spm_path(std::move(spm_path)),
              m_image_type_list(std::move(image_type_list)) {}

    SpmReader(std::string spm_path, const std::vector<SpmImage::ImageType> &image_type_list)
            : m_spm_path(std::move(spm_path)) {
        for (auto &image_type: image_type_list) {
            if ((int) image_type < 0 || (int) image_type >= SpmImage::image_type_str.size()) continue;
            m_image_type_list.emplace_back(SpmImage::image_type_str[(int) image_type]);
        }
    }

    ~SpmReader() = default;

public:
    bool readSpm() {
        if (m_spm_path.empty() || m_image_type_list.empty()) return false;

        // spm file text structure: Head list + Image list x n
        std::unordered_map<std::string, std::string> spm_file_text_map = loadSpmFileTextMap();
        if (spm_file_text_map.size() < 2) {
            return false;
        }

        // Parse SPM file text to file head attributes
        parseFileHeadAttributes(spm_file_text_map.at("Head"));

        // Parse SPM file text to image attributes and load SPM image data
        for (auto &spm_file_text: spm_file_text_map) {
            if (spm_file_text.first != "Head") {
                SpmImage spm_image;
                spm_image.parseImageAttributes(spm_file_text.second);

                // TODO: 1. 完善 Image Type
                if (spm_file_text.first == "Height Sensor") {
                    spm_image.setZScale(spm_file_text.second, "Sens. ZsensSens", m_sens_ZsensSens);
                    std::vector<char> byte_data = loadSpmImageData(spm_image);
                    bool status = spm_image.setImageData(byte_data);
                    if (!status) return false;
                } else if (spm_file_text.first == "Amplitude Error") {
                    spm_image.setZScale(spm_file_text.second, "Sens. Amplitude Error", m_sens_AmplitudeError);
                    std::vector<char> byte_data = loadSpmImageData(spm_image);
                    bool status = spm_image.setImageData(byte_data);
                    if (!status) return false;
                } else {
                    continue;
                }

                m_image_list.emplace(spm_file_text.first, std::move(spm_image));
            }
        }

        if (m_image_list.empty()) return false;

        return true;
    }

    // Suitable for single channel
    bool isImageAvailableSingle() {
        return m_image_list.begin() != m_image_list.end();
    }

    bool isImageAvailable(const std::string &image_type) {
        return m_image_list.find(image_type) != m_image_list.end();
    }

    bool isImageAvailable(const SpmImage::ImageType &image_type) {
        return m_image_list.find(SpmImage::image_type_str[(int) image_type]) != m_image_list.end();
    }

    // Suitable for single channel
    SpmImage &getImageSingle() {
        return m_image_list.begin()->second;
    }

    SpmImage &getImage(const std::string &image_type) {
        return m_image_list.at(image_type);
    }

    SpmImage &getImage(const SpmImage::ImageType &image_type) {
        return m_image_list.at(SpmImage::image_type_str[(int) image_type]);
    }

    // Suitable for single channel
    std::vector<std::vector<double>> &getImageRealDataSingle() {
        return m_image_list.begin()->second.getRealData();
    }

    std::vector<std::vector<double>> &getImageRealData(const std::string &image_type) {
        return m_image_list.at(image_type).getRealData();
    }

    std::vector<std::vector<double>> &getImageRealData(const SpmImage::ImageType &image_type) {
        return m_image_list.at(SpmImage::image_type_str[(int) image_type]).getRealData();
    }

private:
    std::unordered_map<std::string, std::string> loadSpmFileTextMap() {
        std::ifstream spm_file(m_spm_path);
        if (!spm_file.is_open()) {
            std::cout << "Failed to open SPM file: " << m_spm_path << std::endl;
            return {};
        }

        std::unordered_map<std::string, std::string> text_map;
        std::string text, line;
        while (std::getline(spm_file, line)) {
            if (line.substr(0, 2) == "\\*") {
                if (line == m_file_list_end_str) {  // end, last Image list
                    std::string image_type = getStringFromTextByRegex(image_type_regex, text);
                    auto image_type_it = std::find(m_image_type_list.begin(), m_image_type_list.end(), image_type);
                    if (image_type_it != m_image_type_list.end()) {  // 是需要的图像
                        text_map.emplace(image_type, text);
                    }
                    break;
                }
                if (line == m_ciao_image_list_str) {
                    if (text_map.empty()) {  // Head list
                        text_map.emplace("Head", text);
                    } else {  // !text_map.empty(), Image list
                        std::string image_type = getStringFromTextByRegex(image_type_regex, text);
                        auto image_type_it = std::find(m_image_type_list.begin(), m_image_type_list.end(), image_type);
                        if (image_type_it != m_image_type_list.end()) {  // 是需要的图像
                            text_map.emplace(image_type, text);
                        }
                    }
                    text.clear();
                }
            }
            text.append(line);
            text.append("\n");
        }

        spm_file.close();

        return text_map;
    }

    std::vector<char> loadSpmImageData(SpmImage &spm_image) {
        std::ifstream spm_file(m_spm_path, std::ios::binary);

        if (!spm_file.is_open()) {
            std::cout << "Failed to open SPM file: " << m_spm_path << std::endl;
            return {};
        }

        std::vector<char> jump_over(spm_image.getDataOffset());
        spm_file.read(jump_over.data(), spm_image.getDataOffset());

        std::vector<char> image_data(spm_image.getDataLength());
        spm_file.read(image_data.data(), spm_image.getDataLength());

        spm_file.close();

        return image_data;
    }

    void parseFileHeadAttributes(std::string &spm_file_text) {
        // Can be modified: 可添加需要的属性
        m_scan_size = getIntFromTextByRegex(scan_size_regex, spm_file_text);

        m_sens_ZsensSens = getDoubleFromTextByRegex(sens_ZsensSens_regex, spm_file_text);
        m_sens_AmplitudeError = getDoubleFromTextByRegex(sens_AmplitudeError_regex, spm_file_text);
    }

public:
    // Can be modified: 可添加需要的属性正则表达式
    const std::string image_type_regex = R"(\@2:Image Data: S \[.*?\] \"(.*?)\")";
    const std::string scan_size_regex = R"(\Scan Size: (\d+(\.\d+)?) nm)";

    const std::string sens_ZsensSens_regex = R"(\@Sens. ZsensSens: V (\d+(\.\d+)?) nm/V)";
    const std::string sens_AmplitudeError_regex = R"(\@Sens. Amplitude Error: V (\d+(\.\d+)?) nm/V)";

private:
    std::string m_spm_path;
    std::vector<std::string> m_image_type_list;

    const std::string m_file_list_end_str = "\\*File list end";
    const std::string m_ciao_image_list_str = "\\*Ciao image list";

    // File head general attributes
    // Can be modified: 可添加需要的属性
    unsigned int m_scan_size{};

    // File head special attributes
    // Can be modified: 可添加需要的属性
    double m_sens_ZsensSens{};
    double m_sens_AmplitudeError{};

    // Image
    std::unordered_map<std::string, SpmImage> m_image_list;
};


int spmReaderExample0() {
    // ------------ single channel ------------

    SpmReader spm_reader_single("xxx.spm");  // default - Height Sensor
    // SpmReader spm_reader_single("xxx.spm", SpmImage::ImageType::HeightSensor);
    // SpmReader spm_reader_single("xxx.spm", "Height Sensor");

    if (!spm_reader_single.readSpm()) {
        std::cout << "Reading spm file error!" << std::endl;
        return -1;
    }

    if (spm_reader_single.isImageAvailableSingle()) {
        auto &height_image = spm_reader_single.getImageSingle();
        // auto &height_image = spm_reader_single.getImage(SpmImage::ImageType::HeightSensor);
        // auto &height_image = spm_reader_single.getImage("Height Sensor");
        auto &height_real_data = height_image.getRealData();
        std::cout << height_real_data[0][0] << std::endl;
        std::cout << height_real_data[height_image.getRows() - 1][0] << std::endl;
    }

    // ------------------------------------

    // ------------ multi channel ------------

    SpmReader spm_reader_multi("xxx.spm",
                               std::vector<SpmImage::ImageType>{
                                       SpmImage::ImageType::HeightSensor,
                                       SpmImage::ImageType::AmplitudeError});
    // SpmReader spm_reader_multi("xxx.spm",
    //                            std::vector<std::string>{"Height Sensor", "Amplitude Error"});

    if (!spm_reader_multi.readSpm()) {
        std::cout << "Reading spm file error!" << std::endl;
        return -1;
    }

    if (spm_reader_multi.isImageAvailable(SpmImage::ImageType::HeightSensor)) {
        auto &height_image = spm_reader_multi.getImage(SpmImage::ImageType::HeightSensor);
        // auto &height_image = spm_reader_multi.getImage("Height Sensor");
        auto &height_real_data = height_image.getRealData();
        std::cout << height_real_data[0][0] << std::endl;
        std::cout << height_real_data[height_image.getRows() - 1][0] << std::endl;
    }

    if (spm_reader_multi.isImageAvailable(SpmImage::ImageType::AmplitudeError)) {
        auto &error_image = spm_reader_multi.getImage(SpmImage::ImageType::AmplitudeError);
        // auto &error_image = spm_reader_multi.getImageSingle("Amplitude Error");
        auto &error_real_data = error_image.getRealData();
        std::cout << error_real_data[0][0] << std::endl;
        std::cout << error_real_data[error_image.getRows() - 1][0] << std::endl;
    }

    // ------------------------------------

    return 0;
}


#endif //SPM_READER_HPP
