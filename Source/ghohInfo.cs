using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace ghoh
{
    public class ghohInfo : GH_AssemblyInfo
    {
        public override string Name => "ghoh";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("aa004dcf-7721-4914-b140-72033b11a62c");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}