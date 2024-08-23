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
            Console.WriteLine("括号内的内容：");
            foreach (string content in extractedContents)
            {
                Console.WriteLine(content);
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
}
