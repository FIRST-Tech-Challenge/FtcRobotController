#pragma once
#if(__cplusplus >= 201703L) || (_MSVC_LANG >= 201703L)
    #define DEPTHAI_NODISCARD [[nodiscard]]
    #if(defined(_MSC_VER)) || (defined(__has_include) && __has_include(<filesystem>)) || (__cplusplus >= 202002L)
        #include <filesystem>
    #endif
#else
    #define DEPTHAI_NODISCARD
#endif
#include <string>

namespace dai {

// TODO C++20 char8_t
// TODO test if caller works when replace "dai::Path" -> "std::filesystem::path"
// TODO test if can `using dai::Path = std::filesystem::path` on C++17 to completely use STL

/**
 * @brief Represents paths on a filesystem; accepts utf-8, Windows utf-16 wchar_t, or
 *        std::filesystem::path.
 *
 *        It is suitable for direct use with OS APIs.
 *        Features are limited to character-set conversion of paths. It is not
 *        intended as a full replacement for std::filesystem::path
 *
 */
class Path {
   public:
#if defined(_WIN32) && defined(_MSC_VER)
    using value_type = wchar_t;  ///< character used by native-encoding of filesystem
#else
    using value_type = char;  ///< character used by native-encoding of filesystem
#endif
    using string_type = std::basic_string<value_type>;

    Path() = default;
    ~Path() = default;
    Path(const Path&) = default;
    Path(Path&&) = default;
    Path& operator=(const Path&) = default;
    Path& operator=(Path&&) = default;

    /**
     * @brief Construct Path object from source.
     *
     * @param source native-encoding character sequence; no conversion
     */
    Path(string_type&& source) noexcept : _nativePath(std::move(source)) {}

    /**
     * @brief Construct Path object from source.
     *
     * @param source native-encoding character sequence; no conversion
     */
    Path(const string_type& source) : _nativePath(source) {}

    /**
     * @brief Construct Path object from source.
     *
     * @param source pointer to null-terminated native-encoding character sequence; no conversion
     */
    Path(const value_type* source) : _nativePath(string_type(source)) {}

#if defined(__cpp_lib_filesystem)
    /**
     * @brief Construct Path object from source.
     *
     * @param source std::filesystem::path; conversion managed by std::filesystem::path
     */
    Path(const std::filesystem::path& source) : _nativePath(source) {}

    #if defined(__cpp_lib_char8_t)
    /**
     * @brief Construct Path object from source.
     *
     * @param source std::u8string; conversion managed by std::filesystem::path
     */
    Path(const std::u8string& source) : Path(std::filesystem::path(source)) {}

    /**
     * @brief Construct Path object from source.
     *
     * @param source pointer to null-terminated char8_t character sequence; conversion managed by std::filesystem::path
     */
    Path(const char8_t* source) : Path(std::filesystem::path(source)) {}
    #endif
#endif

#if defined(_WIN32) && defined(_MSC_VER)
   private:
    static std::wstring convert_utf8_to_wide(const std::string& utf8string);

   public:
    /**
     * @brief Construct Path object from source.
     *
     * @param source utf-8 character sequence; converts to Windows utf-16
     */
    Path(const std::string& source) : _nativePath(convert_utf8_to_wide(source)) {}

    /**
     * @brief Construct Path object from source.
     *
     * @param source pointer to null-terminated utf-8 character sequence; converts to Windows utf-16
     */
    Path(const char* source) : _nativePath(convert_utf8_to_wide(std::string(source))) {}

    /**
     * @brief Get path in narrow multibyte representation in the current C locale.
     *
     *        Will throw exception if there is no valid conversion.
     *        Could be used to get the narrow ascii representation.
     *
     * @return std::string in narrow multibyte representation
     */
    std::string string() const;

    #if defined(__cpp_lib_char8_t)
    /**
     * @brief Get path in utf-8.
     *
     *        Will throw exception if there is no valid conversion.
     *
     * @return std::u8string in utf-8
     */
    std::u8string u8string() const {
        return std::filesystem::path(_nativePath).u8string();
    }
    #else
    /**
     * @brief Get path in utf-8.
     *
     *        Will throw exception if there is no valid conversion.
     *
     * @return std::string in utf-8
     */
    std::string u8string() const;
    #endif
#else
    /**
     * @brief Get path in native-encoding string; no conversion.
     *
     * @return std::string
     */
    std::string string() const {
        return _nativePath;
    }

    #if defined(__cpp_lib_char8_t)
    /**
     * @brief Get path in utf-8.
     *
     * @return std::u8string in utf-8
     */
    std::u8string u8string() const {
        return std::filesystem::path(_nativePath).u8string();
    }
    #else
    /**
     * @brief Get path in utf-8.
     *
     * @return std::string in utf-8
     */
    std::string u8string() const {
        return _nativePath;
    }
    #endif
#endif

    /**
     * @brief Implicitly convert to native-encoding string, suitable for use with OS APIs
     *
     * @return std::string of utf-8 on most OSs, std::wstring of utf-16 on Windows
     */
    operator string_type() const noexcept {
        return _nativePath;
    }

    /**
     * @brief Returns native-encoding string by const reference, suitable for use with OS APIs
     *
     * @return const std::string& of utf-8 on most OSs, const std::wstring& of utf-16 on Windows
     */
    const string_type& native() const noexcept {
        return _nativePath;
    }

    /**
     * @brief Observes if path is empty (contains no string/folders/filename)
     *
     * @return bool true if the path is empty, false otherwise
     */
    // TODO add back DEPTHAI_NODISCARD once sphinx fixes are in place
    bool empty() const noexcept {
        return _nativePath.empty();
    }

   private:
    string_type _nativePath;
};

}  // namespace dai
