#ifndef SPM_READER_HPP
#define SPM_READER_HPP

#include <iostream>
#include <fstream>
#include <windows.h>
#include <regex>
#include <cmath>
#include <utility>
#include <string>
#include <vector>
#include <unordered_map>


// example
int spmReaderExample0();


class StringOperations {
protected:
    /**
     * @brief string 转 wstring
     *
     * @param str src string
     * @param CodePage The encoding format of the file calling this function. CP_ACP for gbk, CP_UTF8 for utf-8.
     * @return dst string
     */
    static std::wstring string2wstring(const std::string &str, _In_ UINT CodePage = CP_ACP) {
        int len = MultiByteToWideChar(CodePage, 0, str.c_str(), -1, nullptr, 0);
        std::wstring wstr(len, L'\0');
        MultiByteToWideChar(CodePage, 0, str.c_str(), -1, const_cast<wchar_t *>(wstr.data()), len);
        wstr.resize(wcslen(wstr.c_str()));
        return wstr;
    }

    /**
     * @brief wstring 转 string
     *
     * @param wstr src wstring
     * @param CodePage The encoding format of the file calling this function. CP_ACP for gbk, CP_UTF8 for utf-8.
     * @return dst string
     */
    static std::string wstring2string(const std::wstring &wstr, _In_ UINT CodePage = CP_ACP) {
        int len = WideCharToMultiByte(CodePage, 0, wstr.c_str(), -1, nullptr, 0, nullptr, nullptr);
        std::string str(len, '\0');
        WideCharToMultiByte(CodePage, 0, wstr.c_str(), -1, const_cast<char *>(str.data()), len, nullptr, nullptr);
        str.resize(strlen(str.c_str()));
        return str;
    }

    static std::string doubleToDecimalString(double value, int decimal_num) {
        double temp = value;
        int digits = 0;
        int sign_bit = value >= 0 ? 1 : -1;

        if (sign_bit == 1) {
            while (temp >= sign_bit) {
                digits += 1;
                temp /= 10;
            }

            if (digits == 0) digits = 1;
        } else {
            digits += 1;

            while (temp <= sign_bit) {
                digits += 1;
                temp /= 10;
            }

            if (digits == 1) digits = 2;
        }

        return std::to_string(value).substr(0, digits + decimal_num + 1);
    }
};


class SpmRegexParse {
public:
    SpmRegexParse() = default;

    ~SpmRegexParse() = default;

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

    static int getIntFromTextByRegexToNM(const std::string &regex_string, std::string &spm_file_text) {
        std::regex pattern(regex_string);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 3) {
            if (matches[2].str() == "nm") {
                return std::stoi(matches[1].str());
            } else if (matches[2].str() == "um") {
                return (int) (std::stod(matches[1].str()) * 1000);
            } else {  // error
                return 0;
            }
        } else {
            return 0;
        }
    }

    static long long getLongLongFromTextByRegexToNM(const std::string &regex_string, std::string &spm_file_text) {
        std::regex pattern(regex_string);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 3) {
            if (matches[2].str() == "nm") {
                return std::stoll(matches[1].str());
            } else if (matches[2].str() == "um") {
                return (long long) (std::stod(matches[1].str()) * 1000);
            } else if (matches[2].str() == "mm") {
                return (long long) (std::stoll(matches[1].str()) * 1000 * 1000);
            } else {  // error
                return 0;
            }
        } else {
            return 0;
        }
    }

    static bool replaceIntFromTextByRegex(const std::string &regex_string, std::string &spm_file_text, int new_value) {
        std::regex pattern(regex_string);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 2) {
            auto match_str_begin_pos = spm_file_text.find(matches[0].str());
            std::string old_value_str = matches[1].str();
            auto old_value_str_begin_pos = spm_file_text.find(old_value_str, match_str_begin_pos);
            spm_file_text = spm_file_text.substr(0, old_value_str_begin_pos) + std::to_string(new_value) +
                            spm_file_text.substr(old_value_str_begin_pos + old_value_str.size());
            return true;
        } else {
            return false;
        }
    }

    static double
    replaceDoubleFromTextByRegex(const std::string &regex_string, std::string &spm_file_text, double new_value) {
        std::regex pattern(regex_string);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 2) {
            auto match_str_begin_pos = spm_file_text.find(matches[0].str());
            std::string old_value_str = matches[1].str();
            auto old_value_str_begin_pos = spm_file_text.find(old_value_str, match_str_begin_pos);
            spm_file_text = spm_file_text.substr(0, old_value_str_begin_pos) + std::to_string(new_value) +
                            spm_file_text.substr(old_value_str_begin_pos + old_value_str.size());
            return true;
        } else {
            return false;
        }
    }
};


class SpmImage : public SpmRegexParse {
public:
    explicit SpmImage(int scan_size)
            : m_scan_size(scan_size) {}

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

    void setZScale(std::string &spm_file_text, std::string &head_text) {
        std::pair<double, std::string> z_scale_info = getZScaleInfoFromTextByRegex(spm_file_text);

        m_z_scale = z_scale_info.first;

        std::string z_scale_sens_regex = R"(\@)" + z_scale_info.second + R"(: V (\d+(\.\d+)?) .*)";
        m_z_scale_sens = getDoubleFromTextByRegex(z_scale_sens_regex, head_text);
    }

    bool setImageData(std::vector<char> byte_data) {
        // set byte data
        m_byte_data = std::move(byte_data);

        // set raw data
        if (m_bytes_per_pixel == 2) {
            std::vector<short> raw_data_16;
            raw_data_16.assign(reinterpret_cast<const short *>(m_byte_data.data()),
                               reinterpret_cast<const short *>(m_byte_data.data() + m_byte_data.size()));
            m_raw_data.assign(raw_data_16.begin(), raw_data_16.end());
        } else if (m_bytes_per_pixel == 4) {
            m_raw_data.assign(reinterpret_cast<const int *>(m_byte_data.data()),
                              reinterpret_cast<const int *>(m_byte_data.data() + m_byte_data.size()));
        } else {
            return false;
        }

        // calc real data
        m_real_data.clear();
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

    std::vector<char> &getByteData() { return m_byte_data; }

    std::vector<int> &getRawData() { return m_raw_data; }

    std::vector<std::vector<double>> &getRealData() { return m_real_data; }

    int getRows() const { return (int) m_number_of_lines; }

    int getCols() const { return (int) m_samps_per_line; }

    int getBytesPerPixel() const { return (int) m_bytes_per_pixel; }

    int getScanSize() const { return m_scan_size; }

    double getZScaleSens() const { return m_z_scale_sens; }

private:
    static std::pair<double, std::string> getZScaleInfoFromTextByRegex(std::string &spm_file_text) {
        std::string z_scale_regex = R"(\@2:Z scale: V \[(.*?)\] \(.*?\) (\d+\.\d+) (.*))";
        std::regex pattern(z_scale_regex);
        std::smatch matches;
        if (std::regex_search(spm_file_text, matches, pattern) && matches.size() >= 4) {
            double value = std::stod(matches[2].str());

            std::string unit = matches[3].str();
            if (unit == "mV") value /= 1000;  // convert mV to V uniformly

            return {value, matches[1].str()};
        } else {
            return {0, ""};
        }
    }

public:
    // TODO: 1. 完善 Image Type
    enum class ImageType {
        Height,
        HeightSensor,
        HeightTrace,
        HeightRetrace,
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

    int m_scan_size{};
    double m_z_scale{};
    double m_z_scale_sens{};

    // Image data
    std::vector<char> m_byte_data;
    std::vector<int> m_raw_data;  // uniformly converted to 4 bytes (int)
    std::vector<std::vector<double>> m_real_data;
};

// TODO: 1. 完善 Image Type
const std::vector<std::string> SpmImage::image_type_str = std::vector<std::string>{
        "Height", "Height Sensor", "Height Trace", "Height Retrace", "Amplitude Error"
};


class SpmReader : public SpmRegexParse, StringOperations {
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
                SpmImage spm_image((int) m_scan_size);
                spm_image.parseImageAttributes(spm_file_text.second);
                spm_image.setZScale(spm_file_text.second, spm_file_text_map.at("Head"));
                std::vector<char> byte_data = loadSpmImageData(spm_image);
                bool status = spm_image.setImageData(byte_data);
                if (!status) return false;

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

    long long getEngageXPosNM() const { return m_engage_x_pos_nm; }

    long long getEngageYPosNM() const { return m_engage_y_pos_nm; }

    int getXOffsetNM() const { return m_x_offset_nm; }

    int getYOffsetNM() const { return m_y_offset_nm; }

    bool saveSpmByte(const std::string &spm_output_path) {
        return saveSpmByte(spm_output_path, m_image_type_list);
    }

    bool saveSpmByte(const std::string &spm_output_path,
                     const std::vector<std::string> &image_type_list) {
        if (spm_output_path.empty()) {
            std::cout << "The output SPM file path is empty." << std::endl;
            return false;
        }

        if (spm_output_path == m_spm_path) {
            std::cout << "The output SPM file path is the same as this SPM file path." << std::endl;
            return false;
        }

        if (image_type_list.empty()) {
            std::cout << "The image type list is empty." << std::endl;
            return false;
        }

        for (auto &image_type: image_type_list) {
            if (!isImageAvailable(image_type)) {
                std::cout << "The image type " << image_type << " is not available." << std::endl;
                return false;
            }
        }

        // 输出文件头并获取文件头的长度
        int head_data_length = buildSpmFileHead(m_spm_path, spm_output_path, image_type_list);
        if (head_data_length == 0) {
            std::cout << "There are no required images in the SPM template file." << std::endl;
            return false;
        }

        // 获取输出文件当前的占用空间
        int current_size = getSpmFileCurrentSize(spm_output_path);
        if (current_size == 0) return false;

        // 填充 空字节 和 图像数据
        return appendSpmFileData(spm_output_path, image_type_list, head_data_length, current_size);
    }

private:
    std::unordered_map<std::string, std::string> loadSpmFileTextMap() {

#ifdef _MSC_VER
        FILE *spm_file = nullptr;
        errno_t err = _wfopen_s(&spm_file, string2wstring(m_spm_path).c_str(), L"r");
#else
        FILE *spm_file = _wfopen(string2wstring(m_spm_path).c_str(), L"r");
#endif

        if (!spm_file) {
            std::cout << "Failed to open SPM file: " << m_spm_path << std::endl;
            return {};
        }

        std::unordered_map<std::string, std::string> text_map;
        std::string text, line;
        std::wstring line_w;
        wchar_t buffer[1024];
        while (fgetws(buffer, sizeof(buffer) / sizeof(buffer[0]), spm_file)) {
            line_w = buffer;
            line = wstring2string(line_w);
            line.assign(line.begin(), line.end() - 1);  // 去除 '\n'
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

        fclose(spm_file);

        return text_map;
    }

    std::vector<char> loadSpmImageData(SpmImage &spm_image) {

#ifdef _MSC_VER
        FILE *spm_file = nullptr;
        errno_t err = _wfopen_s(&spm_file, string2wstring(m_spm_path).c_str(), L"rb");
#else
        FILE *spm_file = _wfopen(string2wstring(m_spm_path).c_str(), L"rb");
#endif

        if (!spm_file) {
            std::cout << "Failed to open SPM file: " << m_spm_path << std::endl;
            return {};
        }

        std::vector<char> jump_over(spm_image.getDataOffset());
        fread(jump_over.data(), 1, spm_image.getDataOffset(), spm_file);

        std::vector<char> image_data(spm_image.getDataLength());
        fread(image_data.data(), 1, spm_image.getDataLength(), spm_file);

        fclose(spm_file);

        return image_data;
    }

    void parseFileHeadAttributes(std::string &spm_file_text) {
        // Can be modified: 可添加需要的属性
        m_scan_size = getIntFromTextByRegex(scan_size_regex, spm_file_text);
        m_engage_x_pos_nm = getLongLongFromTextByRegexToNM(engage_x_pos_regex, spm_file_text);
        m_engage_y_pos_nm = getLongLongFromTextByRegexToNM(engage_y_pos_regex, spm_file_text);
        m_x_offset_nm = getIntFromTextByRegex(x_offset_regex, spm_file_text);
        m_y_offset_nm = getIntFromTextByRegex(y_offset_regex, spm_file_text);
    }

    int buildSpmFileHead(const std::string &spm_tmpl_path, const std::string &spm_output_path,
                         const std::vector<std::string> &image_type_list) {
#ifdef _MSC_VER
        FILE *spm_tmpl_file = nullptr;
        errno_t err_index = _wfopen_s(&spm_tmpl_file, string2wstring(spm_tmpl_path).c_str(), L"r");
#else
        FILE *spm_tmpl_file = _wfopen(string2wstring(spm_tmpl_path).c_str(), L"r");
#endif

        if (!spm_tmpl_file) {
            std::cout << "Failed to open SPM file: " << spm_tmpl_path << std::endl;
            return 0;
        }

        std::wofstream spm_output_file(spm_output_path);

        bool is_head = true;
        int output_image_count = 0;
        int head_data_length = 0, image_data_offset = 0;

        std::string tmpl_text, tmpl_line;
        std::wstring tmpl_text_w, tmpl_line_w;
        wchar_t tmpl_buffer[1024];

        while (fgetws(tmpl_buffer, sizeof(tmpl_buffer) / sizeof(tmpl_buffer[0]), spm_tmpl_file)) {
            tmpl_line_w = tmpl_buffer;
            tmpl_line = wstring2string(tmpl_line_w);

            if (tmpl_line == (m_ciao_image_list_str + "\n") || tmpl_line == (m_file_list_end_str + "\n")) {
                if (is_head) {  // head
                    // get head data length and calculate next image head data offset
                    head_data_length = getIntFromTextByRegex(m_data_length_str, tmpl_text);
                    image_data_offset += head_data_length;

                    // output head text
                    spm_output_file << tmpl_text_w;

                    is_head = false;
                } else {  // image
                    // determine whether it is the required image
                    std::string this_image_type = getStringFromTextByRegex(image_type_regex, tmpl_text);
                    if (std::find(image_type_list.begin(), image_type_list.end(), this_image_type) !=
                        image_type_list.end()) {  // 是需要的图像
                        // set image head data offset and calculate next image head data offset
                        replaceIntFromTextByRegex(m_data_offset_str, tmpl_text, image_data_offset);
                        image_data_offset += getIntFromTextByRegex(m_data_length_str, tmpl_text);

                        // output head text
                        tmpl_text_w = string2wstring(tmpl_text);
                        spm_output_file << tmpl_text_w;

                        output_image_count += 1;
                        if (output_image_count == image_type_list.size()) break;
                    }
                }

                tmpl_text.clear();
                tmpl_text_w.clear();
            }

            tmpl_text.append(tmpl_line);
            tmpl_text_w.append(tmpl_line_w);
        }

        fclose(spm_tmpl_file);
        spm_output_file.close();

        // no required image
        if (output_image_count == 0) head_data_length = 0;

        return head_data_length;
    }

    static int getSpmFileCurrentSize(const std::string &spm_path) {
        std::ifstream spm_file(spm_path, std::ios::binary | std::ios::ate);
        if (!spm_file.is_open()) {
            std::cout << "Failed to open output file: " << spm_path << std::endl;
            return 0;
        }

        std::streamsize current_size = spm_file.tellg();  // 获取当前文件大小

        spm_file.close();

        return (int) current_size;
    }

    bool appendSpmFileData(const std::string &spm_path, const std::vector<std::string> &image_type_list,
                           int head_data_length, int current_size) {
        std::ofstream spm_file(spm_path, std::ios::binary | std::ios::app);
        if (!spm_file.is_open()) {
            std::cout << "Failed to open output file: " << spm_path << std::endl;
            return false;
        }

        if (current_size < head_data_length) {
            // 填充一个 '0x1A'
            char fill_byte = '\x1A';
            spm_file.write(&fill_byte, sizeof(fill_byte));

            // 使用 '0x00' 填充直到达到所需大小
            std::streamsize bytes_to_append = head_data_length - current_size - 1;
            char zero_byte = '\0';
            for (std::streamsize i = 0; i < bytes_to_append; ++i) {
                spm_file.write(&zero_byte, sizeof(zero_byte));
            }
        }

        for (auto &i : image_type_list) {
            auto &image = m_image_list.at(i);
            std::vector<char> &byte_data = image.getByteData();

            if (byte_data.empty()) {
                spm_file.close();
                return false;
            }

            for (char a_byte : byte_data) {
                spm_file.write(&a_byte, sizeof(a_byte));
            }
        }

        spm_file.close();

        return true;
    }

public:
    // Can be modified: 可添加需要的属性正则表达式
    const std::string image_type_regex = R"(\@2:Image Data: S \[.*?\] \"(.*?)\")";
    const std::string scan_size_regex = R"(\Scan Size: (\d+(\.\d+)?) nm)";
    const std::string engage_x_pos_regex = R"(\\Engage X Pos: ([0-9.-]*) ([num]*))";
    const std::string engage_y_pos_regex = R"(\\Engage Y Pos: ([0-9.-]*) ([num]*))";
    const std::string x_offset_regex = R"(\\X Offset: ([0-9.-]*) ([num]*))";
    const std::string y_offset_regex = R"(\\Y Offset: ([0-9.-]*) ([num]*))";

private:
    std::string m_spm_path;
    std::vector<std::string> m_image_type_list;

    const std::string m_file_list_end_str = "\\*File list end";
    const std::string m_ciao_image_list_str = "\\*Ciao image list";
    const std::string m_data_length_str = R"(\Data length: (\d+))";
    const std::string m_data_offset_str = R"(\Data offset: (\d+))";

    // File head general attributes
    // Can be modified: 可添加需要的属性
    unsigned int m_scan_size{};
    long long m_engage_x_pos_nm{};
    long long m_engage_y_pos_nm{};
    int m_x_offset_nm{};
    int m_y_offset_nm{};

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
