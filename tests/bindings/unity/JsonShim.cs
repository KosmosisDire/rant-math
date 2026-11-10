// Enough of System.Text.Json for tests/bindings/csharp/Program.cs to run inside Unity, which ships none.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace System.Text.Json
{
    public enum JsonValueKind { Undefined, Object, Array, String, Number, True, False, Null }

    public readonly struct JsonProperty
    {
        public readonly string Name;
        public readonly JsonElement Value;
        public JsonProperty(string name, JsonElement value) { Name = name; Value = value; }
    }

    public sealed class JsonDocument : IDisposable
    {
        public JsonElement RootElement;
        public static JsonDocument Parse(string text)
        {
            int at = 0;
            return new JsonDocument { RootElement = Read(text, ref at) };
        }
        public void Dispose() { }

        static void Space(string s, ref int at)
        {
            while (at < s.Length && char.IsWhiteSpace(s[at])) at++;
        }

        static string Text(string s, ref int at)
        {
            int start = ++at;
            while (s[at] != '"') at++;
            return s.Substring(start, at++ - start);
        }

        static JsonElement Read(string s, ref int at)
        {
            Space(s, ref at);
            char c = s[at];
            if (c == '{')
            {
                at++;
                var members = new List<JsonProperty>();
                Space(s, ref at);
                while (s[at] != '}')
                {
                    Space(s, ref at);
                    string key = Text(s, ref at);
                    Space(s, ref at);
                    at++;
                    members.Add(new JsonProperty(key, Read(s, ref at)));
                    Space(s, ref at);
                    if (s[at] == ',') at++;
                    Space(s, ref at);
                }
                at++;
                return new JsonElement { ValueKind = JsonValueKind.Object, Members = members };
            }
            if (c == '[')
            {
                at++;
                var items = new List<JsonElement>();
                Space(s, ref at);
                while (s[at] != ']')
                {
                    items.Add(Read(s, ref at));
                    Space(s, ref at);
                    if (s[at] == ',') at++;
                    Space(s, ref at);
                }
                at++;
                return new JsonElement { ValueKind = JsonValueKind.Array, Items = items };
            }
            if (c == '"') return new JsonElement { ValueKind = JsonValueKind.String, Str = Text(s, ref at) };
            if (string.CompareOrdinal(s, at, "true", 0, 4) == 0) { at += 4; return new JsonElement { ValueKind = JsonValueKind.True }; }
            if (string.CompareOrdinal(s, at, "false", 0, 5) == 0) { at += 5; return new JsonElement { ValueKind = JsonValueKind.False }; }
            if (string.CompareOrdinal(s, at, "null", 0, 4) == 0) { at += 4; return new JsonElement { ValueKind = JsonValueKind.Null }; }
            int start = at;
            while (at < s.Length && "+-0123456789.eE".IndexOf(s[at]) >= 0) at++;
            return new JsonElement { ValueKind = JsonValueKind.Number, Number = double.Parse(s.Substring(start, at - start), CultureInfo.InvariantCulture) };
        }
    }

    public class JsonElement
    {
        public JsonValueKind ValueKind;
        internal List<JsonProperty> Members;
        internal List<JsonElement> Items;
        internal string Str;
        internal double Number;

        public JsonElement GetProperty(string name)
        {
            foreach (var m in Members) if (m.Name == name) return m.Value;
            throw new KeyNotFoundException(name);
        }
        public bool TryGetProperty(string name, out JsonElement value)
        {
            foreach (var m in Members)
            {
                if (m.Name == name) { value = m.Value; return true; }
            }
            value = null;
            return false;
        }
        public string GetString() => Str;
        public double GetDouble() => Number;
        public int GetInt32() => (int)Number;
        public bool GetBoolean() => ValueKind == JsonValueKind.True;
        public int GetArrayLength() => Items.Count;
        public JsonElement this[int i] => Items[i];
        public IEnumerable<JsonElement> EnumerateArray() => Items;
        public IEnumerable<JsonProperty> EnumerateObject() => Members;
        public string GetRawText() => ValueKind.ToString();
    }

    public class JsonSerializerOptions { public bool IncludeFields; public int NumberHandling; }
    public static class JsonSerializer
    {
        public static string Serialize(object v, JsonSerializerOptions o) => v?.ToString();
    }
}

namespace System.Text.Json.Serialization
{
    public static class JsonNumberHandling { public const int AllowNamedFloatingPointLiterals = 0; }
}
