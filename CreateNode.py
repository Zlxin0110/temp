using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;

class Program
{
    static void Main()
    {
        string input = "{1}-{4}*{8}";

        // 检查括号是否匹配
        if (AreBracketsMatched(input))
        {
            Console.WriteLine("括号匹配！");
            
            // 抽取括号内的内容
            List<string> extractedContents = ExtractContents(input);
            Console.WriteLine("括号内的内容判断：");
            foreach (string content in extractedContents)
            {
                if (IsNumeric(content))
                {
                    Console.WriteLine($"'{content}' 是数字");
                }
                else if (IsAlphabetic(content))
                {
                    Console.WriteLine($"'{content}' 是字母");
                }
                else
                {
                    Console.WriteLine($"'{content}' 包含其他字符");
                }
            }
        }
        else
        {
            Console.WriteLine("括号不匹配！");
        }
    }

    static bool AreBracketsMatched(string input)
    {
        Stack<char> stack = new Stack<char>();

        foreach (char c in input)
        {
            if (c == '{')
            {
                stack.Push(c);
            }
            else if (c == '}')
            {
                if (stack.Count == 0 || stack.Pop() != '{')
                {
                    return false;
                }
            }
        }

        return stack.Count == 0;
    }

    static List<string> ExtractContents(string input)
    {
        List<string> contents = new List<string>();
        Regex regex = new Regex(@"\{([^}]*)\}");
        MatchCollection matches = regex.Matches(input);

        foreach (Match match in matches)
        {
            contents.Add(match.Groups[1].Value);
        }

        return contents;
    }

    static bool IsNumeric(string content)
    {
        foreach (char c in content)
        {
            if (!char.IsDigit(c))
            {
                return false;
            }
        }
        return true;
    }

    static bool IsAlphabetic(string content)
    {
        foreach (char c in content)
        {
            if (!char.IsLetter(c))
            {
                return false;
            }
        }
        return true;
    }
}
