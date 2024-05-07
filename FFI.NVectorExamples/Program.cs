using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FFI.NVectorExamples
{
    class Program
    {
        static void Main(string[] args)
        {
            System.Threading.Thread.CurrentThread.CurrentCulture = System.Globalization.CultureInfo.InvariantCulture;
            var examples = new Examples();
            examples.Example1();
            examples.Example2();
            examples.Example3();
            examples.Example4();
            examples.Example5();
            examples.Example6();
            examples.Example7();
            examples.Example8();
            examples.Example9();
            examples.Example10();
            Console.ReadLine();
        }
    }
}
