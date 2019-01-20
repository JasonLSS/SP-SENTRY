'判断文件夹是否存在
Dim fso, wsShell
Set fso = CreateObject("Scripting.FileSystemObject")
Set wsShell = WScript.CreateObject("wscript.shell")
currDir = wsShell.CurrentDirectory
Set folder = fso.GetFolder(currDir)
for each subfolder in folder.SubFolders
	Set prj = fso.GetFolder(subfolder.path)
	Set	files = prj.Files
	Set reg = new RegExP '正则表达式
	reg.Pattern = "\w*.uvprojx"
	for each f in files
		if reg.Test(f.Name) then
			wsShell.run f.path
			wscript.quit
		end if
	next
next
Msgbox "Please check that there are no Chinese characters in path."
