// json.h - v1.0 - Single-header public domain JSON library
//
// The intent of this library is to provide a simple and convenient in-memory
// representation of JSON values, and provide both parsing and printing
// functionality for round-trip conversions between JSON-encoded text and in-
// memory values.
//
// The original author of this software is Sterling Orsten, and its permanent
// home is <http://github.com/sgorsten/json/>. If you find this software
// useful, an acknowledgement in your source text and/or product documentation
// is appreciated, but not required.



// This is free and unencumbered software released into the public domain.
// 
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
// 
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
// 
// For more information, please refer to <http://unlicense.org/>



#ifndef JSON_H
#define JSON_H
#include <cstdint>   // For int32_t, etc.
#include <cassert>   // For assert(...)
#include <array>     // For std::array
#include <vector>    // For std::vector
#include <map>       // For std::map
#include <stdexcept> // For std::runtime_error
#include <sstream>   // For std::ostringstream
#include <fstream>  // used by our json::parsefile

namespace json 
{
    enum class kind { string, number, object, array, true_, false_, null }; // Describes one of seven distinct categories of JSON values
    class value;                                                            // Stores a JSON value
    typedef std::vector<value> array;                                       // Stores a JSON array (an ordered list of values)
    typedef std::map<std::string, value> object;                            // Stores a JSON object (a collection of name/value pairs)
    struct parse_error : std::runtime_error { parse_error(const std::string & what) : runtime_error("json parse error - " + what) {} };

    value parse(const std::string & text); // throws parse_error
	value parsefile(std::string filename);
	bool is_json_number(const std::string & num);

    class value
    {
        kind                    k;
        std::string             str;  // Contents of String or Number value
        object                  obj;  // Fields of Object value
        array                   arr;  // Elements of Array value

                                value(kind k, std::string str)              : k(k), str(move(str)) {}
    public:
                                value()                                     : k(kind::null) {}                     // Default construct null
                                value(const char * s)                       : value(kind::string, s) {}            // Construct string from C-string
                                value(std::string s)                        : value(kind::string, move(s)) {}      // Construct string from std::string
        template<class N, class = typename std::enable_if<std::is_arithmetic<N>::value, void>::type>
                                value(N n)                                  : k(kind::number) { std::ostringstream ss; ss << +n; str = ss.str(); }
                                value(object o)                             : k(kind::object), obj(move(o)) {}     // Construct object from vector<pair<string,value>> (TODO: Assert no duplicate keys)
                                value(array a)                              : k(kind::array), arr(move(a)) {}      // Construct array from vector<value>
                                value(bool b)                               : k(b ? kind::true_ : kind::false_) {} // Construct true or false from boolean
                                value(std::nullptr_t)                       : k(kind::null) {}                     // Construct null from nullptr

        const value &           operator [] (size_t index) const            { const static value null; return index < arr.size() ? arr[index] : null; }
        const value &           operator [] (int index) const               { const static value null; return index < 0 ? null : operator[](static_cast<size_t>(index)); }
        const value &           operator [] (const char * key) const        { for (auto & kvp : obj) if (kvp.first == key) return kvp.second; const static value null; return null; }
        const value &           operator [] (const std::string & key) const { return operator[](key.c_str()); }

        kind                    get_kind() const                            { return k; }
        const std::string &     get_contents() const                        { return str; }    // Contents, if a String, JSON format number, if a Number, empty otherwise
        const object &          get_object() const                          { return obj; }    // Name/value pairs, if an Object, empty otherwise
        const array &           get_array() const                           { return arr; }    // Values, if an Array, empty otherwise

        bool                    is_string() const                           { return k == kind::string; }
        bool                    is_number() const                           { return k == kind::number; }
        bool                    is_object() const                           { return k == kind::object; }
        bool                    is_array() const                            { return k == kind::array; }
        bool                    is_true() const                             { return k == kind::true_; }
        bool                    is_false() const                            { return k == kind::false_; }
        bool                    is_null() const                             { return k == kind::null; }

        bool                    bool_or_default(bool def) const             { return is_true() ? true : is_false() ? false : def; }
        std::string             string_or_default(const char * def) const   { return k == kind::string ? str : def; }
        template<class T> T     number_or_default(T def) const              { if (!is_number()) return def; auto val = +T(); std::istringstream(str) >> val; return static_cast<T>(val); }

        std::string             string() const                              { return string_or_default(""); } // Value, if a String, empty otherwise
        template<class T> T     number() const                              { return number_or_default(T()); } // Value, if a Number, empty otherwise

        void                    set(const char * key, json::value val)      { assert(is_object()); obj[key] = val; }

        static value            from_number(std::string num)                { assert(is_json_number(num)); return value(kind::number, move(num)); }
    };

    inline bool operator == (const value & a, const value & b) { return a.get_kind() == b.get_kind() && a.get_contents() == b.get_contents() && a.get_object() == b.get_object() && a.get_array() == b.get_array(); }
    inline bool operator != (const value & a, const value & b) { return !(a == b); }

    std::ostream & operator << (std::ostream & out, const value & val);
    std::ostream & operator << (std::ostream & out, const array & arr);
    std::ostream & operator << (std::ostream & out, const object & obj);

    template<class T> struct tabbed_ref { const T & value; int tabWidth, indent; };
    template<class T> tabbed_ref<T> tabbed(const T & value, int tabWidth, int indent = 0) { return{ value, tabWidth, indent }; }
    std::ostream & operator << (std::ostream & out, tabbed_ref<value> val);
    std::ostream & operator << (std::ostream & out, tabbed_ref<array> arr);
    std::ostream & operator << (std::ostream & out, tabbed_ref<object> obj);
}

inline json::value to_json(bool b) { return b; }  
inline json::value to_json(const std::string & s) { return s; }
template<class T> typename std::enable_if<std::is_arithmetic<T>::value, json::value>::type to_json(T n) { return n; }
template<class T, int N> json::value to_json(const T (& a)[N]) { json::array r(N); for(int i=0; i<N; ++i) r[i] = to_json(a[i]); return r; }
template<class T, size_t N> json::value to_json(const std::array<T,N> & a) { json::array r(N); for(size_t i=0; i<N; ++i) r[i] = to_json(a[i]); return r; }
template<class T> json::value to_json(const std::vector<T> & v) { json::array r(v.size()); for(size_t i=0; i<v.size(); ++i) r[i] = to_json(v[i]); return r; }
template<class T> json::value to_json(const std::map<std::string, T> &map) { json::object r; for (auto &e : map)  r[e.first]=to_json(e.second);  return r; }
struct field_encoder { json::object & o; template<class T, class... TS> void operator () (const char * name, const T & field, TS...) { o.emplace(name, to_json(field)); } };
template<class T> typename std::enable_if<std::is_class<T>::value, json::value>::type to_json(const T & o) { json::object r; visit_fields(const_cast<T &>(o), field_encoder{r}); return r; }
    
inline void from_json(bool & b, const json::value & val) { b = val.is_true(); }
inline void from_json(std::string & s, const json::value & val) { s = val.string(); }
template<class T> typename std::enable_if<std::is_arithmetic<T>::value>::type from_json(T & n, const json::value & val) { n = val.number<T>(); }
template<class T, int N> void from_json(T (& a)[N], const json::value & val) { for(int i=0; i<N; ++i) from_json(a[i], val[i]); }
template<class T, size_t N> void from_json(std::array<T,N> & a, const json::value & val) { for(size_t i=0; i<N; ++i) from_json(a[i], val[i]); }
template<class T> void from_json(std::vector<T> & v, const json::value & val) { v.resize(val.get_array().size()); for(size_t i=0; i<v.size(); ++i) from_json(v[i], val[i]); }
template<class T> void from_json(std::map<std::string, T> & m, const json::value & val) { for (auto &e : val.get_object()) { from_json(m[e.first], e.second); } }
struct field_decoder { const json::value & v; template<class T, class... TS> void operator () (const char * name, T & field, TS...) { from_json(field, v[name]); } };
template<class T> typename std::enable_if<std::is_class<T>::value>::type from_json(T & o, const json::value & val) { visit_fields(o, field_decoder{val}); }   


#include <algorithm>
#include <regex>

namespace json 
{ 
	inline std::ostream & print_escaped(std::ostream & out, const std::string & str)
    {
        // Escape sequences for ", \, and control characters, 0 indicates no escaping needed
        static const char * escapes[256] = {
            "\\u0000", "\\u0001", "\\u0002", "\\u0003", "\\u0004", "\\u0005", "\\u0006", "\\u0007",
            "\\b", "\\t", "\\n", "\\u000B", "\\f", "\\r", "\\u000E", "\\u000F",
            "\\u0010", "\\u0011", "\\u0012", "\\u0013", "\\u0014", "\\u0015", "\\u0016", "\\u0017",
            "\\u0018", "\\u0019", "\\u001A", "\\u001B", "\\u001C", "\\u001D", "\\u001E", "\\u001F",
            0, 0, "\\\"", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "\\\\", 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "\\u007F"
        };
        out << '"';
        for (uint8_t ch : str)
        {
            if (escapes[ch]) out << escapes[ch];
            else out << ch;
        }
        return out << '"';
    }

	inline std::ostream & operator << (std::ostream & out, const array & arr)
    {
        int i = 0;
        out << '[';
        for (auto & val : arr) out << (i++ ? "," : "") << val;
        return out << ']';
    }

	inline std::ostream & operator << (std::ostream & out, const object & obj)
    {
        int i = 0;
        out << '{';
        for (auto & kvp : obj)
        {
            print_escaped(out << (i++ ? "," : ""), kvp.first) << ':' << kvp.second;
        }
        return out << '}';
    }

	inline std::ostream & operator << (std::ostream & out, const value & val)
    {
        if (val.is_null()) return out << "null";
        else if (val.is_false()) return out << "false";
        else if (val.is_true()) return out << "true";
        else if (val.is_string()) return print_escaped(out, val.get_contents());
        else if (val.is_number()) return out << val.get_contents();
        else if (val.is_array()) return out << val.get_array();
        else return out << val.get_object();
    }

    static std::ostream & indent(std::ostream & out, int space, int n = 0)
    {
        if (n) out << ',';
        out << '\n';
        for (int i = 0; i < space; ++i) out << ' ';
        return out;
    }

	inline std::ostream & operator << (std::ostream & out, tabbed_ref<array> arr)
    {
        if (std::none_of(begin(arr.value), end(arr.value), [](const value & val) { return val.is_array() || val.is_object(); })) return out << arr.value;
        else
        {
            int space = arr.indent + arr.tabWidth, i = 0;
            out << '[';
            for (auto & val : arr.value) indent(out, space, i++) << tabbed(val, arr.tabWidth, space);
            return indent(out, arr.indent) << ']';
        }
    }

	inline std::ostream & operator << (std::ostream & out, tabbed_ref<object> obj)
    {
        if (obj.value.empty()) return out << "{}";
        else
        {
            int space = obj.indent + obj.tabWidth, i = 0;
            out << '{';
            for (auto & kvp : obj.value)
            {
                print_escaped(indent(out, space, i++), kvp.first) << ": " << tabbed(kvp.second, obj.tabWidth, space);
            }
            return indent(out, obj.indent) << '}';
        }
    }

	inline std::ostream & operator << (std::ostream & out, tabbed_ref<value> val)
    {
        if (val.value.is_array()) return out << tabbed(val.value.get_array(), val.tabWidth, val.indent);
        else if (val.value.is_object()) return out << tabbed(val.value.get_object(), val.tabWidth, val.indent);
        else return out << val.value;
    }

	inline bool is_json_number(const std::string & num)
    {
    #ifdef __GNUC__
        auto it=begin(num); if(it == end(num)) return false;                   // String cannot be empty
        if(*it == '-') ++it; if(it == end(num)) return false;                  // Discard optional - at start of string

        if(*it == '0') ++it;                                                   // Whole number part can be 0
        else if(isdigit(*it)) { while(it != end(num) && isdigit(*it)) ++it; }  // or [1-9][0-9]*
        else return false;                                                     // but anything else fails the match
        if(it == end(num)) return true;                                        // Acceptable to stop here

        if(*it == '.')                                                         // If there is a .
        {
            ++it; if(it == end(num)) return false;                             // We need at least one more digit
            while(it != end(num) && isdigit(*it)) ++it;                        // Skip all digits
            if(it == end(num)) return true;                                    // Acceptable to stop here
        } 

        if(*it == 'e' || *it == 'E')                                           // If there is an e or an E
        {
            ++it; if(it == end(num)) return false;                             // We need the exponent term
            if(*it == '+' || *it == '-') ++it;                                 // Skip +/-
            if(it == end(num)) return false;                                   // We still need the exponent term
            while(it != end(num) && isdigit(*it)) ++it;                        // Skip all digits
            if(it == end(num)) return true;                                    // Acceptable to stop here
        }

        return false;                                                          // Anything left over fails the match
    #else
        static const std::regex regex(R"(-?(0|([1-9][0-9]*))((\.[0-9]+)?)(((e|E)((\+|-)?)[0-9]+)?))");
	    return std::regex_match(begin(num), end(num), regex);
    #endif
    }

    static uint16_t decode_hex(char ch) {
        if (ch >= '0' && ch <= '9') return ch - '0';
        if (ch >= 'A' && ch <= 'F') return 10 + ch - 'A';
        if (ch >= 'a' && ch <= 'f') return 10 + ch - 'a';
        throw parse_error(std::string("invalid hex digit: ") + ch);
    }

    static std::string decode_string(std::string::const_iterator first, std::string::const_iterator last)
    {
        if (std::any_of(first, last, iscntrl)) throw parse_error("control character found in string literal");
        if (std::find(first, last, '\\') == last) return std::string(first, last); // No escape characters, use the string directly
        std::string s; s.reserve(last - first); // Reserve enough memory to hold the entire string
        for (; first < last; ++first)
        {
            if (*first != '\\') s.push_back(*first);
            else switch (*(++first))
            {
            case '"': s.push_back('"'); break;
            case '\\': s.push_back('\\'); break;
            case '/': s.push_back('/'); break;
            case 'b': s.push_back('\b'); break;
            case 'f': s.push_back('\f'); break;
            case 'n': s.push_back('\n'); break;
            case 'r': s.push_back('\r'); break;
            case 't': s.push_back('\t'); break;
            case 'u':
                if (first + 5 > last) throw parse_error("incomplete escape sequence: " + std::string(first - 1, last));
                else
                {
                    uint16_t val = (decode_hex(first[1]) << 12) | (decode_hex(first[2]) << 8) | (decode_hex(first[3]) << 4) | decode_hex(first[4]);
                    if (val < 0x80) s.push_back(static_cast<char>(val)); // ASCII codepoint, no translation needed
                    else if (val < 0x800) // 2-byte UTF-8 encoding
                    {
                        s.push_back(0xC0 | ((val >> 6) & 0x1F)); // Leading byte: 5 content bits
                        s.push_back(0x80 | ((val >> 0) & 0x3F)); // Continuation byte: 6 content bits
                    }
                    else // 3-byte UTF-8 encoding (16 content bits, sufficient to store all \uXXXX patterns)
                    {
                        s.push_back(0xE0 | ((val >> 12) & 0x0F)); // Leading byte: 4 content bits
                        s.push_back(0x80 | ((val >> 6) & 0x3F)); // Continuation byte: 6 content bits
                        s.push_back(0x80 | ((val >> 0) & 0x3F)); // Continuation byte: 6 content bits
                    }
                    first += 4;
                }
                break;
            default: throw parse_error("invalid escape sequence");
            }
        }
        return s;
    }

    struct token { char type; std::string value; token(char type, std::string value = std::string()) : type(type), value(move(value)) {} };

    struct parse_state
    {
        std::vector<token>::iterator it, last;

        bool match_and_discard(char type) { if (it->type != type) return false; ++it; return true; }
        void discard_expected(char type, const char * what) { if (!match_and_discard(type)) throw parse_error(std::string("Syntax error: Expected ") + what); }

        value parse_value()
        {
            auto token = it++;
            switch (token->type)
            {
            case 'n': return nullptr;
            case 'f': return false;
            case 't': return true;
            case '"': return token->value;
            case '#': return value::from_number(token->value);
            case '[':
                if (match_and_discard(']')) return array{};
                else
                {
                    array arr;
                    while (true)
                    {
                        arr.push_back(parse_value());
                        if (match_and_discard(']')) return arr;
                        discard_expected(',', ", or ]");
                    }
                }
            case '{':
                if (match_and_discard('}')) return object{};
                else
                {
                    object obj;
                    while (true)
                    {
                        auto name = move(it->value);
                        discard_expected('"', "string");
                        discard_expected(':', ":");
                        obj.emplace(move(name), parse_value());
                        if (match_and_discard('}')) { return obj; }
                        discard_expected(',', ", or }");
                    }
                }
            default: throw parse_error("Expected value");
            }
        }
    };

	inline std::vector<token> parse_tokens(const std::string & text)
    {
        std::vector<token> tokens;
        auto it = begin(text);
        while (true)
        {
            it = std::find_if_not(it, end(text), isspace); // Skip whitespace
            if (it == end(text))
            {
                tokens.emplace_back('$');
                return tokens;
            }
            switch (*it)
            {
            case '[': case ']': case ',':
            case '{': case '}': case ':':
                tokens.push_back({ *it++ });
                break;
            case '"':
                {
                    auto it2 = ++it;
                    for (; it2 < end(text); ++it2)
                    {
                        if (*it2 == '"') break;
                        if (*it2 == '\\') ++it2;
                    }
                    if (it2 < end(text))
                    {
                        tokens.emplace_back('"', decode_string(it, it2));
                        it = it2 + 1;
                    }
                    else throw parse_error("String missing closing quote");
                }
                break;
            case '-': case '0': case '1': case '2':
            case '3': case '4': case '5': case '6':
            case '7': case '8': case '9':
                {
                    auto it2 = std::find_if_not(it, end(text), [](char ch) { return isalnum(ch) || ch == '+' || ch == '-' || ch == '.'; });
                    auto num = std::string(it, it2);
                    if (!is_json_number(num)) throw parse_error("Invalid number: " + num);
                    tokens.emplace_back('#', move(num));
                    it = it2;
                }
                break;
            default:
                if (isalpha(*it))
                {
                    auto it2 = std::find_if_not(it, end(text), isalpha);
                    if (std::equal(it, it2, "true")) tokens.emplace_back('t');
                    else if (std::equal(it, it2, "false")) tokens.emplace_back('f');
                    else if (std::equal(it, it2, "null")) tokens.emplace_back('n');
                    else throw parse_error("Invalid token: " + std::string(it, it2));
                    it = it2;
                }
                else throw parse_error("Invalid character: \'" + std::string(1, *it) + '"');
            }
        }
    }

	inline value parse(const std::string & text)
    {
        auto tokens = parse_tokens(text);
        parse_state p = { begin(tokens), end(tokens) };
        auto val = p.parse_value();
        p.discard_expected('$', "end-of-stream");
        return val;
    }
	inline value parsefile(std::string filename)
	{
		std::ifstream file(filename, std::ios::binary | std::ios::ate);
		if (!file.is_open())
			throw(std::exception((std::string("File Not Found: ") + filename).c_str()));
		auto len = file.tellg();
		file.seekg(0, std::ios::beg);
		std::string mem((size_t)len, ' ');
		file.read(&mem[0], len);
		file.close();
		return parse(mem);
	}
}

#endif  JSON_H

