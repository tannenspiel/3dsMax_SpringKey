	-- installer.ms
myCategory = "TannenspielScripts"
myName = "SpringKey"
	
if queryBox "Install SpringKey script?"	then
(
	if ((doesFileExist (getDir #usermacros)) and (doesFileExist (getDir #userScripts))) then
	(
		-- variables of files and paths
		scriptPath = #userScripts
		macrosPath = #usermacros
		iconPath = #usericons
		myFolder = myCategory
		myMSE = myName + ".ms"
		myMCR = myName + ".mcr"
		myICON16 = myName + "_16i.bmp"
		myICON24 = myName + "_24i.bmp"
		
		
		-- get temp folder path of unpacked installer.ms
		installerPath = getFilenamePath (getSourceFileName())
		-- set temp folder path to create temp .mcr
		tempPathMCR = installerPath + "\\" + myName + ".mcr"

		myMSEpath = installerPath + "\\" + myMSE
		myICON16path = installerPath + "\\" + myICON16
		myICON24path = installerPath + "\\" + myICON24
		

		--makeDir (getDir #userScripts + "\\WASD")
		makeDir (getDir scriptPath + "\\" + myFolder)
		copyFile myMSEpath (getDir scriptPath + "\\" + myFolder + "\\" + myMSE)
		copyFile myICON16path (getDir iconPath + "\\" + myICON16)
		copyFile myICON24path (getDir iconPath + "\\" + myICON24)
		--copyFile myMCR (getDir macrosPath + "\\" + myMCR)
		
		-- run .mcr directly from mzp
		--fileIn myMCR 

		-- run .mcr from usermacros folder
		-- fileIn (getDir macrosPath + "\\" + filenameFromPath myMCR)
		
		
		myMacroStr = "macroScript " + myName + "\n"
		myCategoryStr = "category:" + "\"" + myCategory + "\"" + "\n"
		myTooltipStr = "tooltip:" + "\"" + myName + "\"" + "\n"
		msePathStr = "(" + "\n" + "\t" + "fileIn (getDir #userScripts + " + "\"" + "\\" + "\\" + myCategory+ "\\" + "\\" + myMSE + "\"" + ")" + "\n" + ")"
		myBtnStr = "buttonText:" + "\"" + myName + "\"" + "\n"
		myIconStr = "Icon: #(\"" + myName + "\", 1)" + "\n"
		
		
		
		-- create .mcr in temp folder
		outputFile = createFile (tempPathMCR)

		format "%%%%%%" myMacroStr myCategoryStr myTooltipStr myBtnStr myIconStr msePathStr to:outputFile
		close outputFile
		-- run .mcr in temp folder
		fileIn tempPathMCR		
		
	)
	--messageBox ((getDir macrosPath + "\\" + myCategory + "-" + myMCR))
	--messageBox ((getDir scriptPath + myFolder + "\\" + myMSE))
	-- check if .mse .mcr are installed
	if ((doesFileExist (getDir macrosPath + "\\" + myCategory + "-" + myMCR)) and (doesFileExist (getDir scriptPath + "\\" + myFolder + "\\" + myMSE))) then
	(
		messageBox ("Script SpringKey successfully installed. You can find out it:\n
		- Menu Customize\n
		- Customize User Interface\n
		- Toolbars\n
		- Category " + myCategory)
	)	
)
