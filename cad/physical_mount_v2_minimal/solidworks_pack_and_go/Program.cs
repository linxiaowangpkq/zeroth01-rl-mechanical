using System;
using System.IO;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System.Runtime.InteropServices;

internal static class Program
{
    [STAThread]
    private static int Main(string[] args)
    {
        try
        {
            return Run(args);
        }
        catch (Exception error)
        {
            Console.Error.WriteLine("ERROR_TYPE=" + error.GetType().FullName);
            try { Console.Error.WriteLine("ERROR_MESSAGE=" + error.Message); } catch { }
            Console.Error.WriteLine("ERROR_HRESULT=0x" + error.HResult.ToString("X8"));
            try { Console.Error.WriteLine("ERROR_STACK=" + error.StackTrace); } catch { }
            return 1;
        }
    }

    private static int Run(string[] args)
    {
        if (args.Length != 2)
        {
            Console.Error.WriteLine("usage: SolidWorksPackAndGo <assembly.SLDASM> <output.zip>");
            return 64;
        }

        string assemblyPath = Path.GetFullPath(args[0]);
        string zipPath = Path.GetFullPath(args[1]);
        if (!File.Exists(assemblyPath))
            throw new FileNotFoundException(assemblyPath);

        Type type = Type.GetTypeFromProgID("SldWorks.Application");
        if (type == null)
            throw new InvalidOperationException("SldWorks.Application ProgID is unavailable");
        object created = Activator.CreateInstance(type);
        if (created == null)
            throw new InvalidOperationException("cannot create or attach SolidWorks");
        SldWorks sw = (SldWorks)created;
        sw.Visible = true;

        IModelDoc2 model = sw.GetOpenDocumentByName(assemblyPath) as IModelDoc2;
        int errors = 0;
        int warnings = 0;
        if (model == null)
        {
            model = sw.OpenDoc6(
                assemblyPath,
                (int)swDocumentTypes_e.swDocASSEMBLY,
                (int)swOpenDocOptions_e.swOpenDocOptions_Silent,
                "",
                ref errors,
                ref warnings
            ) as IModelDoc2;
        }
        if (model == null)
            throw new InvalidOperationException(string.Format("cannot open assembly; errors={0}, warnings={1}", errors, warnings));

        IModelDocExtension extension = model.Extension;
        PackAndGo pack = extension.GetPackAndGo();
        pack.IncludeDrawings = false;
        pack.IncludeSimulationResults = false;
        pack.IncludeSuppressed = true;
        pack.IncludeToolboxComponents = true;
        pack.FlattenToSingleFolder = true;

        int documentCount = pack.GetDocumentNamesCount();
        Directory.CreateDirectory(Path.GetDirectoryName(zipPath));
        if (File.Exists(zipPath))
            File.Delete(zipPath);
#pragma warning disable 618
        bool overrideOk = pack.SetSaveToName(true, zipPath);
#pragma warning restore 618
        object statusesObject = extension.SavePackAndGo(pack);
        int[] statuses = statusesObject as int[];
        if (statuses == null)
            statuses = new int[0];

        Console.WriteLine("SOLIDWORKS_REVISION=" + sw.RevisionNumber());
        Console.WriteLine("DOCUMENT_COUNT=" + documentCount);
        Console.WriteLine("OVERRIDE_OK=" + overrideOk);
        Console.WriteLine("SAVE_STATUS_COUNT=" + statuses.Length);
        Console.WriteLine("SAVE_STATUSES=" + string.Join(",", statuses));
        Console.WriteLine("ZIP=" + zipPath);
        Console.WriteLine("ZIP_BYTES=" + (File.Exists(zipPath) ? new FileInfo(zipPath).Length : 0));
        return overrideOk && File.Exists(zipPath) ? 0 : 2;
    }
}
