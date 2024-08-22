# This block is executed when generating an intermediate resource file, not when
# running in CMake configure mode
if(_CMRC_GENERATE_MODE)
    if (INCBIN_ENABLED)

        string(CONFIGURE [[
        #include <incbin/incbin.h>
        namespace cmrc { namespace @NAMESPACE@ { namespace res_chars {
        INCBIN(@SYMBOL@, "@INPUT_FILE@");
        extern const char* const @SYMBOL@_begin = (const char*) g@SYMBOL@Data;
        extern const char* const @SYMBOL@_end = (const char*) (g@SYMBOL@Data + g@SYMBOL@Size);
        }}}
        ]] code)

    else()
        
        # Read in the digits
        file(READ "${INPUT_FILE}" bytes HEX)
        # Format each pair into a character literal. Heuristics seem to favor doing
        # the conversion in groups of five for fastest conversion
        string(REGEX REPLACE "(..)(..)(..)(..)(..)" "'\\\\x\\1','\\\\x\\2','\\\\x\\3','\\\\x\\4','\\\\x\\5'," chars "${bytes}")
        # Since we did this in groups, we have some leftovers to clean up
        string(LENGTH "${bytes}" n_bytes2)
        math(EXPR n_bytes "${n_bytes2} / 2")
        math(EXPR remainder "${n_bytes} % 5") # <-- '5' is the grouping count from above
        set(cleanup_re "$")
        set(cleanup_sub )
        while(remainder)
            set(cleanup_re "(..)${cleanup_re}")
            set(cleanup_sub "'\\\\x\\${remainder}',${cleanup_sub}")
            math(EXPR remainder "${remainder} - 1")
        endwhile()
        if(NOT cleanup_re STREQUAL "$")
            string(REGEX REPLACE "${cleanup_re}" "${cleanup_sub}" chars "${chars}")
        endif()

        string(CONFIGURE [[
        namespace { const char file_array[] = { @chars@ 0 }; }
        namespace cmrc { namespace @NAMESPACE@ { namespace res_chars {
        extern const char* const @SYMBOL@_begin = file_array;
        extern const char* const @SYMBOL@_end = file_array + @n_bytes@;
        }}}
        ]] code)
        
    endif()

    file(WRITE "${OUTPUT_FILE}" "${code}")
    # Exit from the script. Nothing else needs to be processed
    return()
endif()



set(_version 2.0.0)

cmake_minimum_required(VERSION 3.3)
include(CMakeParseArguments)

if(COMMAND cmrc_add_resource_library)
    if(NOT DEFINED _CMRC_VERSION OR NOT (_version STREQUAL _CMRC_VERSION))
        message(WARNING "More than one CMakeRC version has been included in this project.")
    endif()
    # CMakeRC has already been included! Don't do anything
    return()
endif()

set(_CMRC_VERSION "${_version}" CACHE INTERNAL "CMakeRC version. Used for checking for conflicts")

set(_CMRC_SCRIPT "${CMAKE_CURRENT_LIST_FILE}" CACHE INTERNAL "Path to CMakeRC script")

function(_cmrc_normalize_path var)
    set(path "${${var}}")
    file(TO_CMAKE_PATH "${path}" path)
    while(path MATCHES "//")
        string(REPLACE "//" "/" path "${path}")
    endwhile()
    string(REGEX REPLACE "/+$" "" path "${path}")
    set("${var}" "${path}" PARENT_SCOPE)
endfunction()

get_filename_component(_inc_dir "${CMAKE_BINARY_DIR}/_cmrc/include" ABSOLUTE)
set(CMRC_INCLUDE_DIR "${_inc_dir}" CACHE INTERNAL "Directory for CMakeRC include files")
# Let's generate the primary include file
file(MAKE_DIRECTORY "${CMRC_INCLUDE_DIR}/cmrc")
set(hpp_content [==[
#ifndef CMRC_CMRC_HPP_INCLUDED
#define CMRC_CMRC_HPP_INCLUDED

#include <cassert>
#include <functional>
#include <iterator>
#include <list>
#include <map>
#include <mutex>
#include <string>
#include <system_error>
#include <type_traits>

namespace cmrc { namespace detail { struct dummy; } }

#define CMRC_DECLARE(libid) \
    namespace cmrc { namespace detail { \
    struct dummy; \
    static_assert(std::is_same<dummy, ::cmrc::detail::dummy>::value, "CMRC_DECLARE() must only appear at the global namespace"); \
    } } \
    namespace cmrc { namespace libid { \
    cmrc::embedded_filesystem get_filesystem(); \
    } } static_assert(true, "")

namespace cmrc {

class file {
    const char* _begin = nullptr;
    const char* _end = nullptr;

public:
    using iterator = const char*;
    using const_iterator = iterator;
    iterator begin() const noexcept { return _begin; }
    iterator cbegin() const noexcept { return _begin; }
    iterator end() const noexcept { return _end; }
    iterator cend() const noexcept { return _end; }
    std::size_t size() const { return std::distance(begin(), end()); }

    file() = default;
    file(iterator beg, iterator end) noexcept : _begin(beg), _end(end) {}
};

class directory_entry;

namespace detail {

class directory;
class file_data;

class file_or_directory {
    union _data_t {
        class file_data* file_data;
        class directory* directory;
    } _data;
    bool _is_file = true;

public:
    explicit file_or_directory(file_data& f) {
        _data.file_data = &f;
    }
    explicit file_or_directory(directory& d) {
        _data.directory = &d;
        _is_file = false;
    }
    bool is_file() const noexcept {
        return _is_file;
    }
    bool is_directory() const noexcept {
        return !is_file();
    }
    const directory& as_directory() const noexcept {
        assert(!is_file());
        return *_data.directory;
    }
    const file_data& as_file() const noexcept {
        assert(is_file());
        return *_data.file_data;
    }
};

class file_data {
public:
    const char* begin_ptr;
    const char* end_ptr;
    file_data(const file_data&) = delete;
    file_data(const char* b, const char* e) : begin_ptr(b), end_ptr(e) {}
};

inline std::pair<std::string, std::string> split_path(const std::string& path) {
    auto first_sep = path.find("/");
    if (first_sep == path.npos) {
        return std::make_pair(path, "");
    } else {
        return std::make_pair(path.substr(0, first_sep), path.substr(first_sep + 1));
    }
}

struct created_subdirectory {
    class directory& directory;
    class file_or_directory& index_entry;
};

class directory {
    std::list<file_data> _files;
    std::list<directory> _dirs;
    std::map<std::string, file_or_directory> _index;

    using base_iterator = std::map<std::string, file_or_directory>::const_iterator;

public:

    directory() = default;
    directory(const directory&) = delete;

    created_subdirectory add_subdir(std::string name) & {
        _dirs.emplace_back();
        auto& back = _dirs.back();
        auto& fod = _index.emplace(name, file_or_directory{back}).first->second;
        return created_subdirectory{back, fod};
    }

    file_or_directory* add_file(std::string name, const char* begin, const char* end) & {
        assert(_index.find(name) == _index.end());
        _files.emplace_back(begin, end);
        return &_index.emplace(name, file_or_directory{_files.back()}).first->second;
    }

    const file_or_directory* get(const std::string& path) const {
        auto pair = split_path(path);
        auto child = _index.find(pair.first);
        if (child == _index.end()) {
            return nullptr;
        }
        auto& entry  = child->second;
        if (pair.second.empty()) {
            // We're at the end of the path
            return &entry;
        }

        if (entry.is_file()) {
            // We can't traverse into a file. Stop.
            return nullptr;
        }
        // Keep going down
        return entry.as_directory().get(pair.second);
    }

    class iterator {
        base_iterator _base_iter;
        base_iterator _end_iter;
    public:
        using value_type = directory_entry;
        using difference_type = std::ptrdiff_t;
        using pointer = const value_type*;
        using reference = const value_type&;
        using iterator_category = std::input_iterator_tag;

        iterator() = default;
        explicit iterator(base_iterator iter, base_iterator end) : _base_iter(iter), _end_iter(end) {}

        iterator begin() const noexcept {
            return *this;
        }

        iterator end() const noexcept {
            return iterator(_end_iter, _end_iter);
        }

        inline value_type operator*() const noexcept;

        bool operator==(const iterator& rhs) const noexcept {
            return _base_iter == rhs._base_iter;
        }

        bool operator!=(const iterator& rhs) const noexcept {
            return !(*this == rhs);
        }

        iterator operator++() noexcept {
            auto cp = *this;
            ++_base_iter;
            return cp;
        }

        iterator& operator++(int) noexcept {
            ++_base_iter;
            return *this;
        }
    };

    using const_iterator = iterator;

    iterator begin() const noexcept {
        return iterator(_index.begin(), _index.end());
    }

    iterator end() const noexcept {
        return iterator();
    }
};

inline std::string normalize_path(std::string path) {
    while (path.find("/") == 0) {
        path.erase(path.begin());
    }
    while (!path.empty() && (path.rfind("/") == path.size() - 1)) {
        path.pop_back();
    }
    auto off = path.npos;
    while ((off = path.find("//")) != path.npos) {
        path.erase(path.begin() + off);
    }
    return path;
}

using index_type = std::map<std::string, const cmrc::detail::file_or_directory*>;

} // detail

class directory_entry {
    std::string _fname;
    const detail::file_or_directory* _item;

public:
    directory_entry() = delete;
    explicit directory_entry(std::string filename, const detail::file_or_directory& item)
        : _fname(filename)
        , _item(&item)
    {}

    const std::string& filename() const & {
        return _fname;
    }
    std::string filename() const && {
        return std::move(_fname);
    }

    bool is_file() const {
        return _item->is_file();
    }

    bool is_directory() const {
        return _item->is_directory();
    }
};

directory_entry detail::directory::iterator::operator*() const noexcept {
    assert(begin() != end());
    return directory_entry(_base_iter->first, _base_iter->second);
}

using directory_iterator = detail::directory::iterator;

class embedded_filesystem {
    // Never-null:
    const cmrc::detail::index_type* _index;
    const detail::file_or_directory* _get(std::string path) const {
        path = detail::normalize_path(path);
        auto found = _index->find(path);
        if (found == _index->end()) {
            return nullptr;
        } else {
            return found->second;
        }
    }

public:
    explicit embedded_filesystem(const detail::index_type& index)
        : _index(&index)
    {}

    file open(const std::string& path) const {
        auto entry_ptr = _get(path);
        if (!entry_ptr || !entry_ptr->is_file()) {
            throw std::system_error(make_error_code(std::errc::no_such_file_or_directory), path);
        }
        auto& dat = entry_ptr->as_file();
        return file{dat.begin_ptr, dat.end_ptr};
    }

    bool is_file(const std::string& path) const noexcept {
        auto entry_ptr = _get(path);
        return entry_ptr && entry_ptr->is_file();
    }

    bool is_directory(const std::string& path) const noexcept {
        auto entry_ptr = _get(path);
        return entry_ptr && entry_ptr->is_directory();
    }

    bool exists(const std::string& path) const noexcept {
        return !!_get(path);
    }

    directory_iterator iterate_directory(const std::string& path) const {
        auto entry_ptr = _get(path);
        if (!entry_ptr) {
            throw std::system_error(make_error_code(std::errc::no_such_file_or_directory), path);
        }
        if (!entry_ptr->is_directory()) {
            throw std::system_error(make_error_code(std::errc::not_a_directory), path);
        }
        return entry_ptr->as_directory().begin();
    }
};

}

#endif // CMRC_CMRC_HPP_INCLUDED
]==])

set(cmrc_hpp "${CMRC_INCLUDE_DIR}/cmrc/cmrc.hpp" CACHE INTERNAL "")
set(_generate 1)
if(EXISTS "${cmrc_hpp}")
    file(READ "${cmrc_hpp}" _current)
    if(_current STREQUAL hpp_content)
        set(_generate 0)
    endif()
endif()
file(GENERATE OUTPUT "${cmrc_hpp}" CONTENT "${hpp_content}" CONDITION ${_generate})

add_library(cmrc-base INTERFACE)

# Add interface include for users of library
target_include_directories(cmrc-base SYSTEM INTERFACE "$<BUILD_INTERFACE:${CMRC_INCLUDE_DIR}>")

# Signal a basic C++11 feature to require C++11.
target_compile_features(cmrc-base INTERFACE cxx_nullptr)
set_property(TARGET cmrc-base PROPERTY INTERFACE_CXX_EXTENSIONS OFF)
add_library(cmrc::base ALIAS cmrc-base)



### Incbin project addition, this enables much quicker compile times on supported compilers
# https://github.com/graphitemaster/incbin
# Incbin project
# Commit: 8cefe46d5380bf5ae4b4d87832d811f6692aae44

# Create an option to use it (default on, with try_compile checks to make sure it works)
option(CMRC_COMPATIBILITY_MODE "If enabled intermediate character array is generated (Supports all compilers)" OFF)

set(incbin_h_content [==[
/**
 * @file incbin.h
 * @author Dale Weiler
 * @brief Utility for including binary files
 *
 * Facilities for including binary files into the current translation unit and
 * making use from them externally in other translation units.
 */
#ifndef INCBIN_HDR
#define INCBIN_HDR
#include <limits.h>
#if   defined(__AVX512BW__) || \
      defined(__AVX512CD__) || \
      defined(__AVX512DQ__) || \
      defined(__AVX512ER__) || \
      defined(__AVX512PF__) || \
      defined(__AVX512VL__) || \
      defined(__AVX512F__)
# define INCBIN_ALIGNMENT_INDEX 6
#elif defined(__AVX__)      || \
      defined(__AVX2__)
# define INCBIN_ALIGNMENT_INDEX 5
#elif defined(__SSE__)      || \
      defined(__SSE2__)     || \
      defined(__SSE3__)     || \
      defined(__SSSE3__)    || \
      defined(__SSE4_1__)   || \
      defined(__SSE4_2__)   || \
      defined(__neon__)
# define INCBIN_ALIGNMENT_INDEX 4
#elif ULONG_MAX != 0xffffffffu
# define INCBIN_ALIGNMENT_INDEX 3
# else
# define INCBIN_ALIGNMENT_INDEX 2
#endif

/* Lookup table of (1 << n) where `n' is `INCBIN_ALIGNMENT_INDEX' */
#define INCBIN_ALIGN_SHIFT_0 1
#define INCBIN_ALIGN_SHIFT_1 2
#define INCBIN_ALIGN_SHIFT_2 4
#define INCBIN_ALIGN_SHIFT_3 8
#define INCBIN_ALIGN_SHIFT_4 16
#define INCBIN_ALIGN_SHIFT_5 32
#define INCBIN_ALIGN_SHIFT_6 64

/* Actual alignment value */
#define INCBIN_ALIGNMENT \
    INCBIN_CONCATENATE( \
        INCBIN_CONCATENATE(INCBIN_ALIGN_SHIFT, _), \
        INCBIN_ALIGNMENT_INDEX)

/* Stringize */
#define INCBIN_STR(X) \
    #X
#define INCBIN_STRINGIZE(X) \
    INCBIN_STR(X)
/* Concatenate */
#define INCBIN_CAT(X, Y) \
    X ## Y
#define INCBIN_CONCATENATE(X, Y) \
    INCBIN_CAT(X, Y)
/* Deferred macro expansion */
#define INCBIN_EVAL(X) \
    X
#define INCBIN_INVOKE(N, ...) \
    INCBIN_EVAL(N(__VA_ARGS__))

/* Green Hills uses a different directive for including binary data */
#if defined(__ghs__)
#  if (__ghs_asm == 2)
#    define INCBIN_MACRO ".file"
/* Or consider the ".myrawdata" entry in the ld file */
#  else
#    define INCBIN_MACRO "\tINCBIN"
#  endif
#else
#  define INCBIN_MACRO ".incbin"
#endif

#ifndef _MSC_VER
#  define INCBIN_ALIGN \
    __attribute__((aligned(INCBIN_ALIGNMENT)))
#else
#  define INCBIN_ALIGN __declspec(align(INCBIN_ALIGNMENT))
#endif

#if defined(__arm__) || /* GNU C and RealView */ \
    defined(__arm) || /* Diab */ \
    defined(_ARM) /* ImageCraft */
#  define INCBIN_ARM
#endif

#ifdef __GNUC__
/* Utilize .balign where supported */
#  define INCBIN_ALIGN_HOST ".balign " INCBIN_STRINGIZE(INCBIN_ALIGNMENT) "\n"
#  define INCBIN_ALIGN_BYTE ".balign 1\n"
#elif defined(INCBIN_ARM)
/*
 * On arm assemblers, the alignment value is calculated as (1 << n) where `n' is
 * the shift count. This is the value passed to `.align'
 */
#  define INCBIN_ALIGN_HOST ".align " INCBIN_STRINGIZE(INCBIN_ALIGNMENT_INDEX) "\n"
#  define INCBIN_ALIGN_BYTE ".align 0\n"
#else
/* We assume other inline assembler's treat `.align' as `.balign' */
#  define INCBIN_ALIGN_HOST ".align " INCBIN_STRINGIZE(INCBIN_ALIGNMENT) "\n"
#  define INCBIN_ALIGN_BYTE ".align 1\n"
#endif

/* INCBIN_CONST is used by incbin.c generated files */
#if defined(__cplusplus)
#  define INCBIN_EXTERNAL extern "C"
#  define INCBIN_CONST    extern const
#else
#  define INCBIN_EXTERNAL extern
#  define INCBIN_CONST    const
#endif

/**
 * @brief Optionally override the linker section into which data is emitted.
 *
 * @warning If you use this facility, you'll have to deal with platform-specific linker output
 * section naming on your own
 *
 * Overriding the default linker output section, e.g for esp8266/Arduino:
 * @code
 * #define INCBIN_OUTPUT_SECTION ".irom.text"
 * #include "incbin.h"
 * INCBIN(Foo, "foo.txt");
 * // Data is emitted into program memory that never gets copied to RAM
 * @endcode
 */
#if !defined(INCBIN_OUTPUT_SECTION)
#  if defined(__APPLE__)
#    define INCBIN_OUTPUT_SECTION         ".const_data"
#  else
#    define INCBIN_OUTPUT_SECTION         ".rodata"
#  endif
#endif

#if defined(__APPLE__)
/* The directives are different for Apple branded compilers */
#  define INCBIN_SECTION         INCBIN_OUTPUT_SECTION "\n"
#  define INCBIN_GLOBAL(NAME)    ".globl " INCBIN_MANGLE INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME "\n"
#  define INCBIN_INT             ".long "
#  define INCBIN_MANGLE          "_"
#  define INCBIN_BYTE            ".byte "
#  define INCBIN_TYPE(...)
#else
#  define INCBIN_SECTION         ".section " INCBIN_OUTPUT_SECTION "\n"
#  define INCBIN_GLOBAL(NAME)    ".global " INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME "\n"
#  if defined(__ghs__)
#    define INCBIN_INT           ".word "
#  else
#    define INCBIN_INT           ".int "
#  endif
#  if defined(__USER_LABEL_PREFIX__)
#    define INCBIN_MANGLE        INCBIN_STRINGIZE(__USER_LABEL_PREFIX__)
#  else
#    define INCBIN_MANGLE        ""
#  endif
#  if defined(INCBIN_ARM)
/* On arm assemblers, `@' is used as a line comment token */
#    define INCBIN_TYPE(NAME)    ".type " INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME ", %object\n"
#  elif defined(__MINGW32__) || defined(__MINGW64__)
/* Mingw doesn't support this directive either */
#    define INCBIN_TYPE(NAME)
#  else
/* It's safe to use `@' on other architectures */
#    define INCBIN_TYPE(NAME)    ".type " INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME ", @object\n"
#  endif
#  define INCBIN_BYTE            ".byte "
#endif

/* List of style types used for symbol names */
#define INCBIN_STYLE_CAMEL 0
#define INCBIN_STYLE_SNAKE 1

/**
 * @brief Specify the prefix to use for symbol names.
 *
 * By default this is `g', producing symbols of the form:
 * @code
 * #include "incbin.h"
 * INCBIN(Foo, "foo.txt");
 *
 * // Now you have the following symbols:
 * // const unsigned char gFooData[];
 * // const unsigned char *const gFooEnd;
 * // const unsigned int gFooSize;
 * @endcode
 *
 * If however you specify a prefix before including: e.g:
 * @code
 * #define INCBIN_PREFIX incbin
 * #include "incbin.h"
 * INCBIN(Foo, "foo.txt");
 *
 * // Now you have the following symbols instead:
 * // const unsigned char incbinFooData[];
 * // const unsigned char *const incbinFooEnd;
 * // const unsigned int incbinFooSize;
 * @endcode
 */
#if !defined(INCBIN_PREFIX)
#  define INCBIN_PREFIX g
#endif

/**
 * @brief Specify the style used for symbol names.
 *
 * Possible options are
 * - INCBIN_STYLE_CAMEL "CamelCase"
 * - INCBIN_STYLE_SNAKE "snake_case"
 *
 * Default option is *INCBIN_STYLE_CAMEL* producing symbols of the form:
 * @code
 * #include "incbin.h"
 * INCBIN(Foo, "foo.txt");
 *
 * // Now you have the following symbols:
 * // const unsigned char <prefix>FooData[];
 * // const unsigned char *const <prefix>FooEnd;
 * // const unsigned int <prefix>FooSize;
 * @endcode
 *
 * If however you specify a style before including: e.g:
 * @code
 * #define INCBIN_STYLE INCBIN_STYLE_SNAKE
 * #include "incbin.h"
 * INCBIN(foo, "foo.txt");
 *
 * // Now you have the following symbols:
 * // const unsigned char <prefix>foo_data[];
 * // const unsigned char *const <prefix>foo_end;
 * // const unsigned int <prefix>foo_size;
 * @endcode
 */
#if !defined(INCBIN_STYLE)
#  define INCBIN_STYLE INCBIN_STYLE_CAMEL
#endif

/* Style lookup tables */
#define INCBIN_STYLE_0_DATA Data
#define INCBIN_STYLE_0_END End
#define INCBIN_STYLE_0_SIZE Size
#define INCBIN_STYLE_1_DATA _data
#define INCBIN_STYLE_1_END _end
#define INCBIN_STYLE_1_SIZE _size

/* Style lookup: returning identifier */
#define INCBIN_STYLE_IDENT(TYPE) \
    INCBIN_CONCATENATE( \
        INCBIN_STYLE_, \
        INCBIN_CONCATENATE( \
            INCBIN_EVAL(INCBIN_STYLE), \
            INCBIN_CONCATENATE(_, TYPE)))

/* Style lookup: returning string literal */
#define INCBIN_STYLE_STRING(TYPE) \
    INCBIN_STRINGIZE( \
        INCBIN_STYLE_IDENT(TYPE)) \

/* Generate the global labels by indirectly invoking the macro with our style
 * type and concatenating the name against them. */
#define INCBIN_GLOBAL_LABELS(NAME, TYPE) \
    INCBIN_INVOKE( \
        INCBIN_GLOBAL, \
        INCBIN_CONCATENATE( \
            NAME, \
            INCBIN_INVOKE( \
                INCBIN_STYLE_IDENT, \
                TYPE))) \
    INCBIN_INVOKE( \
        INCBIN_TYPE, \
        INCBIN_CONCATENATE( \
            NAME, \
            INCBIN_INVOKE( \
                INCBIN_STYLE_IDENT, \
                TYPE)))

/**
 * @brief Externally reference binary data included in another translation unit.
 *
 * Produces three external symbols that reference the binary data included in
 * another translation unit.
 *
 * The symbol names are a concatenation of `INCBIN_PREFIX' before *NAME*; with
 * "Data", as well as "End" and "Size" after. An example is provided below.
 *
 * @param NAME The name given for the binary data
 *
 * @code
 * INCBIN_EXTERN(Foo);
 *
 * // Now you have the following symbols:
 * // extern const unsigned char <prefix>FooData[];
 * // extern const unsigned char *const <prefix>FooEnd;
 * // extern const unsigned int <prefix>FooSize;
 * @endcode
 */
#define INCBIN_EXTERN(NAME) \
    INCBIN_EXTERNAL const INCBIN_ALIGN unsigned char \
        INCBIN_CONCATENATE( \
            INCBIN_CONCATENATE(INCBIN_PREFIX, NAME), \
            INCBIN_STYLE_IDENT(DATA))[]; \
    INCBIN_EXTERNAL const INCBIN_ALIGN unsigned char *const \
    INCBIN_CONCATENATE( \
        INCBIN_CONCATENATE(INCBIN_PREFIX, NAME), \
        INCBIN_STYLE_IDENT(END)); \
    INCBIN_EXTERNAL const unsigned int \
        INCBIN_CONCATENATE( \
            INCBIN_CONCATENATE(INCBIN_PREFIX, NAME), \
            INCBIN_STYLE_IDENT(SIZE))

/**
 * @brief Include a binary file into the current translation unit.
 *
 * Includes a binary file into the current translation unit, producing three symbols
 * for objects that encode the data and size respectively.
 *
 * The symbol names are a concatenation of `INCBIN_PREFIX' before *NAME*; with
 * "Data", as well as "End" and "Size" after. An example is provided below.
 *
 * @param NAME The name to associate with this binary data (as an identifier.)
 * @param FILENAME The file to include (as a string literal.)
 *
 * @code
 * INCBIN(Icon, "icon.png");
 *
 * // Now you have the following symbols:
 * // const unsigned char <prefix>IconData[];
 * // const unsigned char *const <prefix>IconEnd;
 * // const unsigned int <prefix>IconSize;
 * @endcode
 *
 * @warning This must be used in global scope
 * @warning The identifiers may be different if INCBIN_STYLE is not default
 *
 * To externally reference the data included by this in another translation unit
 * please @see INCBIN_EXTERN.
 */
#ifdef _MSC_VER
#define INCBIN(NAME, FILENAME) \
    INCBIN_EXTERN(NAME)
#else
#define INCBIN(NAME, FILENAME) \
    __asm__(INCBIN_SECTION \
            INCBIN_GLOBAL_LABELS(NAME, DATA) \
            INCBIN_ALIGN_HOST \
            INCBIN_MANGLE INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME INCBIN_STYLE_STRING(DATA) ":\n" \
            INCBIN_MACRO " \"" FILENAME "\"\n" \
            INCBIN_GLOBAL_LABELS(NAME, END) \
            INCBIN_ALIGN_BYTE \
            INCBIN_MANGLE INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME INCBIN_STYLE_STRING(END) ":\n" \
                INCBIN_BYTE "1\n" \
            INCBIN_GLOBAL_LABELS(NAME, SIZE) \
            INCBIN_ALIGN_HOST \
            INCBIN_MANGLE INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME INCBIN_STYLE_STRING(SIZE) ":\n" \
                INCBIN_INT INCBIN_MANGLE INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME INCBIN_STYLE_STRING(END) " - " \
                           INCBIN_MANGLE INCBIN_STRINGIZE(INCBIN_PREFIX) #NAME INCBIN_STYLE_STRING(DATA) "\n" \
            INCBIN_ALIGN_HOST \
            ".text\n" \
    ); \
    INCBIN_EXTERN(NAME)

#endif
#endif
]==])

if (CMRC_COMPATIBILITY_MODE)
    set(HAVE_INCBIN_CAPABILITY FALSE CACHE INTERNAL "")
else()

    get_filename_component(_inc_gen_dir "${CMAKE_BINARY_DIR}/_cmrc/gen/include" ABSOLUTE)
    # Lets generate incbin.h file
    file(MAKE_DIRECTORY "${_inc_gen_dir}/incbin")


    # Create in generate stage
    set(cmrc_incbin_h "${_inc_gen_dir}/incbin/incbin.h" CACHE INTERNAL "")
    set(_incbin_generate 1)
    if(EXISTS "${cmrc_incbin_h}")
        file(READ "${cmrc_incbin_h}" _incbin_current)
        if(_incbin_current STREQUAL incbin_h_content)
            set(_incbin_generate 0)
        endif()
    endif()
    file(GENERATE OUTPUT "${cmrc_incbin_h}" CONTENT "${incbin_h_content}" CONDITION ${_incbin_generate})

    # Tests if incbin assembly directive works for current compiler (otherwise fallback to generating character array)

    if(NOT CMRC_INCBIN_TESTS_DONE)

        # Let's generate the test for incbin.h
        get_filename_component(_tests_gen_dir "${CMAKE_BINARY_DIR}/_cmrc/tests" ABSOLUTE)
        file(MAKE_DIRECTORY "${_tests_gen_dir}")

        # Create a sample input file
        set(test_txt_content [==[
        Hello World!
        ]==])
        file(WRITE "${_tests_gen_dir}/test.txt" "${test_txt_content}")

        # Create a test program source
        string(CONFIGURE [==[
        #define INCBIN_PREFIX g_
        #define INCBIN_STYLE INCBIN_STYLE_SNAKE
        @incbin_h_content@
        namespace test{
            INCBIN(test, "@_tests_gen_dir@/test.txt");
        }
        int main(){
            int sum = 0;
            for(unsigned int i = 0; i<test::g_test_size; i++){
                sum += test::g_test_data[i];
            }
            return sum;
        }
        ]==] test_cpp_content @ONLY)
        file(WRITE "${_tests_gen_dir}/test.cpp" "${test_cpp_content}")


        # Now try to compile
        message(STATUS "Check for working incbin assembly directive")
        try_compile(HAVE_INCBIN_CAPABILITY "${_tests_gen_dir}" "${_tests_gen_dir}/test.cpp"
            CMAKE_FLAGS "-DINCLUDE_DIRECTORIES=${_inc_gen_dir}"
        )
        if(HAVE_INCBIN_CAPABILITY)
            message(STATUS "Check for working incbin assembly directive - works")
        else()
            message(STATUS "Check for working incbin assembly directive - does not work")
            message(STATUS "CMRC - Compatibility mode, creating intermediate character array files")
        endif()

        # Remove tests
        file(REMOVE_RECURSE "${_tests_gen_dir}")

        set(CMRC_INCBIN_TESTS_DONE TRUE CACHE INTERNAL "")

    endif()

endif()

function(cmrc_add_resource_library name)
    set(args ALIAS NAMESPACE)
    cmake_parse_arguments(ARG "" "${args}" "" "${ARGN}")
    # Generate the identifier for the resource library's namespace
    set(ns_re "[a-zA-Z_][a-zA-Z0-9_]*")
    if(NOT DEFINED ARG_NAMESPACE)
        # Check that the library name is also a valid namespace
        if(NOT name MATCHES "${ns_re}")
            message(SEND_ERROR "Library name is not a valid namespace. Specify the NAMESPACE argument")
        endif()
        set(ARG_NAMESPACE "${name}")
    else()
        if(NOT ARG_NAMESPACE MATCHES "${ns_re}")
            message(SEND_ERROR "NAMESPACE for ${name} is not a valid C++ namespace identifier (${ARG_NAMESPACE})")
        endif()
    endif()
    set(libname "${name}")
    # Generate a library with the compiled in character arrays.
    string(CONFIGURE [=[
        #include <cmrc/cmrc.hpp>
        #include <map>
        #include <utility>

        namespace cmrc {
        namespace @ARG_NAMESPACE@ {

        namespace res_chars {
        // These are the files which are available in this resource library
        $<JOIN:$<TARGET_PROPERTY:@libname@,CMRC_EXTERN_DECLS>,
        >
        }

        namespace {

        const cmrc::detail::index_type&
        get_root_index() {
            static cmrc::detail::directory root_directory_;
            static cmrc::detail::file_or_directory root_directory_fod{root_directory_};
            static cmrc::detail::index_type root_index;
            root_index.emplace("", &root_directory_fod);
            struct dir_inl {
                class cmrc::detail::directory& directory;
            };
            dir_inl root_directory_dir{root_directory_};
            (void)root_directory_dir;
            $<JOIN:$<TARGET_PROPERTY:@libname@,CMRC_MAKE_DIRS>,
            >
            $<JOIN:$<TARGET_PROPERTY:@libname@,CMRC_MAKE_FILES>,
            >
            return root_index;
        }

        }

        cmrc::embedded_filesystem get_filesystem() {
            static auto& index = get_root_index();
            return cmrc::embedded_filesystem{index};
        }

        } // @ARG_NAMESPACE@
        } // cmrc
    ]=] cpp_content @ONLY)
    get_filename_component(libdir "${CMAKE_CURRENT_BINARY_DIR}/__cmrc_${name}" ABSOLUTE)
    get_filename_component(lib_tmp_cpp "${libdir}/lib_.cpp" ABSOLUTE)
    string(REPLACE "\n        " "\n" cpp_content "${cpp_content}")
    file(GENERATE OUTPUT "${lib_tmp_cpp}" CONTENT "${cpp_content}")
    get_filename_component(libcpp "${libdir}/lib.cpp" ABSOLUTE)
    add_custom_command(OUTPUT "${libcpp}"
        DEPENDS "${lib_tmp_cpp}" "${cmrc_hpp}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${lib_tmp_cpp}" "${libcpp}"
        COMMENT "Generating ${name} resource loader"
        )
    # Generate the actual static library. Each source file is just a single file
    # with a character array compiled in containing the contents of the
    # corresponding resource file.
    add_library(${name} STATIC ${libcpp})
    
    # Add private include to incbin/incbin.h
    target_include_directories(${name} PRIVATE "${_inc_gen_dir}")

    set_property(TARGET ${name} PROPERTY CMRC_LIBDIR "${libdir}")
    set_property(TARGET ${name} PROPERTY CMRC_NAMESPACE "${ARG_NAMESPACE}")
    target_link_libraries(${name} PUBLIC cmrc::base)
    set_property(TARGET ${name} PROPERTY CMRC_IS_RESOURCE_LIBRARY TRUE)
    if(ARG_ALIAS)
        add_library("${ARG_ALIAS}" ALIAS ${name})
    endif()
    cmrc_add_resources(${name} ${ARG_UNPARSED_ARGUMENTS})
endfunction()

function(_cmrc_register_dirs name dirpath)
    if(dirpath STREQUAL "")
        return()
    endif()
    # Skip this dir if we have already registered it
    get_target_property(registered "${name}" _CMRC_REGISTERED_DIRS)
    if(dirpath IN_LIST registered)
        return()
    endif()
    # Register the parent directory first
    get_filename_component(parent "${dirpath}" DIRECTORY)
    if(NOT parent STREQUAL "")
        _cmrc_register_dirs("${name}" "${parent}")
    endif()
    # Now generate the registration
    set_property(TARGET "${name}" APPEND PROPERTY _CMRC_REGISTERED_DIRS "${dirpath}")
    _cm_encode_fpath(sym "${dirpath}")
    if(parent STREQUAL "")
        set(parent_sym root_directory)
    else()
        _cm_encode_fpath(parent_sym "${parent}")
    endif()
    get_filename_component(leaf "${dirpath}" NAME)
    set_property(
        TARGET "${name}"
        APPEND PROPERTY CMRC_MAKE_DIRS
        "static auto ${sym}_dir = ${parent_sym}_dir.directory.add_subdir(\"${leaf}\")\;"
        "root_index.emplace(\"${dirpath}\", &${sym}_dir.index_entry)\;"
        )
endfunction()

function(cmrc_add_resources name)
    get_target_property(is_reslib ${name} CMRC_IS_RESOURCE_LIBRARY)
    if(NOT TARGET ${name} OR NOT is_reslib)
        message(SEND_ERROR "cmrc_add_resources called on target '${name}' which is not an existing resource library")
        return()
    endif()

    set(options)
    set(args WHENCE PREFIX)
    set(list_args)
    cmake_parse_arguments(ARG "${options}" "${args}" "${list_args}" "${ARGN}")

    if(NOT ARG_WHENCE)
        set(ARG_WHENCE ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    _cmrc_normalize_path(ARG_WHENCE)
    get_filename_component(ARG_WHENCE "${ARG_WHENCE}" ABSOLUTE)

    # Generate the identifier for the resource library's namespace
    get_target_property(lib_ns "${name}" CMRC_NAMESPACE)

    get_target_property(libdir ${name} CMRC_LIBDIR)
    get_target_property(target_dir ${name} SOURCE_DIR)
    file(RELATIVE_PATH reldir "${target_dir}" "${CMAKE_CURRENT_SOURCE_DIR}")
    if(reldir MATCHES "^\\.\\.")
        message(SEND_ERROR "Cannot call cmrc_add_resources in a parent directory from the resource library target")
        return()
    endif()

    foreach(input IN LISTS ARG_UNPARSED_ARGUMENTS)
        _cmrc_normalize_path(input)
        get_filename_component(abs_in "${input}" ABSOLUTE)
        # Generate a filename based on the input filename that we can put in
        # the intermediate directory.
        file(RELATIVE_PATH relpath "${ARG_WHENCE}" "${abs_in}")
        if(relpath MATCHES "^\\.\\.")
            # For now we just error on files that exist outside of the soure dir.
            message(SEND_ERROR "Cannot add file '${input}': File must be in a subdirectory of ${ARG_WHENCE}")
            continue()
        endif()
        if(DEFINED ARG_PREFIX)
            _cmrc_normalize_path(ARG_PREFIX)
        endif()
        if(ARG_PREFIX AND NOT ARG_PREFIX MATCHES "/$")
            set(ARG_PREFIX "${ARG_PREFIX}/")
        endif()
        get_filename_component(dirpath "${ARG_PREFIX}${relpath}" DIRECTORY)
        _cmrc_register_dirs("${name}" "${dirpath}")
        get_filename_component(abs_out "${libdir}/intermediate/${relpath}.cpp" ABSOLUTE)
        # Generate a symbol name relpath the file's character array
        _cm_encode_fpath(sym "${relpath}")
        # Get the symbol name for the parent directory
        if(dirpath STREQUAL "")
            set(parent_sym root_directory)
        else()
            _cm_encode_fpath(parent_sym "${dirpath}")
        endif()
        # Generate the rule for the intermediate source file
        _cmrc_generate_intermediate_cpp(${lib_ns} ${sym} "${abs_out}" "${abs_in}" "${HAVE_INCBIN_CAPABILITY}")
        target_sources(${name} PRIVATE "${abs_out}")
        set_property(TARGET ${name} APPEND PROPERTY CMRC_EXTERN_DECLS
            "// Pointers to ${input}"
            "extern const char* const ${sym}_begin\;"
            "extern const char* const ${sym}_end\;"
            )
        get_filename_component(leaf "${relpath}" NAME)
        set_property(
            TARGET ${name}
            APPEND PROPERTY CMRC_MAKE_FILES
            "root_index.emplace("
            "    \"${ARG_PREFIX}${relpath}\","
            "    ${parent_sym}_dir.directory.add_file("
            "        \"${leaf}\","
            "        res_chars::${sym}_begin,"
            "        res_chars::${sym}_end"
            "    )"
            ")\;"
            )
    endforeach()
endfunction()

function(_cmrc_generate_intermediate_cpp lib_ns symbol outfile infile incbin_enabled)
    add_custom_command(
        # This is the file we will generate
        OUTPUT "${outfile}"
        # These are the primary files that affect the output
        DEPENDS "${infile}" "${_CMRC_SCRIPT}"
        COMMAND
            "${CMAKE_COMMAND}"
                -D_CMRC_GENERATE_MODE=TRUE
                -DNAMESPACE=${lib_ns}
                -DSYMBOL=${symbol}
                "-DINPUT_FILE=${infile}"
                "-DOUTPUT_FILE=${outfile}"
                "-DINCBIN_ENABLED=${incbin_enabled}"
                -P "${_CMRC_SCRIPT}"
        COMMENT "Generating intermediate file for ${infile}"
    )
endfunction()

function(_cm_encode_fpath var fpath)
    string(MAKE_C_IDENTIFIER "${fpath}" ident)
    string(MD5 hash "${fpath}")
    string(SUBSTRING "${hash}" 0 4 hash)
    set(${var} f_${hash}_${ident} PARENT_SCOPE)
endfunction()
