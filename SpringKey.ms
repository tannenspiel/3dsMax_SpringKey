/*
    SpringKey
    Live, non-baked spring/elastic animation controllers for 3ds Max.

    Run this file, then use the SpringKey rollout to apply Position,
    Rotation, and/or Scale spring controllers to selected nodes.
*/

global SpringKey

struct SpringKeyCore
(
    logEnabled = false,
    logFilePath = (try((getFilenamePath (getSourceFileName())) + "SpringKey_debug.log")catch((getDir #scripts) + "\\SpringKey_debug.log")),

    fn setLogging enabled =
    (
        logEnabled = enabled
    ),

    fn clearLog =
    (
        try
        (
            local f = createFile logFilePath
            format "SpringKey debug log\n" to:f
            format "Cleared at: %\n\n" localTime to:f
            close f
        )
        catch()
    ),

    fn logText text =
    (
        if logEnabled do
        (
            try
            (
                local f = openFile logFilePath mode:"a"
                format "%\n" text to:f
                close f
            )
            catch()
        )
    ),

    fn logRotationHeader sourceNode =
    (
        if logEnabled do
        (
            try
            (
                local f = openFile logFilePath mode:"a"
                format "\n[Rotation Eval]\n" to:f
                format "time: % ticks, frame: %\n" (currentTime as integer) ((currentTime as float) / TicksPerFrame) to:f
                format "source: %\n" sourceNode to:f
                close f
            )
            catch()
        )
    ),

    fn logRotationDetails label value =
    (
        if logEnabled do
        (
            try
            (
                local f = openFile logFilePath mode:"a"
                format "%: %\n" label value to:f
                close f
            )
            catch()
        )
    ),

    fn secondsFromTimeValue t =
    (
        (t as float) / 4800.0
    ),

    fn degreesToRadians value =
    (
        value * pi / 180.0
    ),


    fn elasticMultiplier amp freq decay dt matchVelocity bounceMode =
    (
        if dt <= 0.0 then
        (
            0.0
        )
        else
        (
            local a = amp / 200.0
            local f = freq / 30.0
            local d = decay / 10.0
            local wave = (sin (f * dt * 360.0)) / (exp (d * dt))
            if bounceMode do wave = -(abs wave)

            if matchVelocity and f > 0.0001 then
            (
                (a / (2.0 * pi * f)) * wave
            )
            else
            (
                a * wave
            )
        )
    ),

    fn dampingMultiplier amp freq decay dt matchVelocity =
    (
        elasticMultiplier amp freq decay dt matchVelocity false
    ),

    fn scanControllerForPreviousKey c targetTime bestSoFar =
    (
        local best = bestSoFar

        try
        (
            local keyCount = numKeys c
            if keyCount > 0 do for i = 1 to keyCount do
            (
                local kt = getKeyTime c i
                if kt <= targetTime do
                (
                    if best == undefined or kt > best do best = kt
                )
            )
        )
        catch()

        try
        (
            for i = 1 to c.numSubs do
            (
                try
                (
                    local subAnim = c[i]
                    try(best = scanControllerForPreviousKey subAnim targetTime best)catch()
                    if subAnim.controller != undefined do
                    (
                        best = scanControllerForPreviousKey subAnim.controller targetTime best
                    )
                )
                catch()
            )
        )
        catch()

        best
    ),

    fn findPreviousKeyTime ctrl t =
    (
        try
        (
            scanControllerForPreviousKey ctrl t undefined
        )
        catch
        (
            undefined
        )
    ),

    fn findLastKeyTime ctrl =
    (
        try
        (
            scanControllerForPreviousKey ctrl animationRange.end undefined
        )
        catch
        (
            undefined
        )
    ),

    fn springPoint3Offset baseCtrl amp freq decay sampleFrames matchVelocity =
    (
        local t = currentTime

        local keyTime = findPreviousKeyTime baseCtrl t
        if keyTime == undefined then
        (
            [0,0,0]
        )
        else
        (
            local dt = secondsFromTimeValue (t - keyTime)
            local sampleTime = keyTime - sampleFrames
            local sampleDt = secondsFromTimeValue (keyTime - sampleTime)

            if sampleDt <= 0.0 then
            (
                [0,0,0]
            )
            else
            (
                local valueAtKey = at time keyTime baseCtrl.value
                local valueBefore = at time sampleTime baseCtrl.value
                local velocity = (valueAtKey - valueBefore) / sampleDt
                velocity * (dampingMultiplier amp freq decay dt matchVelocity)
            )
        )
    ),

    fn springPoint3OffsetAt baseCtrl evalTime amp freq decay sampleFrames matchVelocity =
    (
        local keyTime = findLastKeyTime baseCtrl
        if keyTime == undefined then
        (
            [0,0,0]
        )
        else if evalTime <= keyTime then
        (
            [0,0,0]
        )
        else
        (
            local dt = secondsFromTimeValue (evalTime - keyTime)
            local sampleTime = keyTime - sampleFrames
            local sampleDt = secondsFromTimeValue (keyTime - sampleTime)

            if sampleDt <= 0.0 then
            (
                [0,0,0]
            )
            else
            (
                local valueAtKey = at time keyTime baseCtrl.value
                local valueBefore = at time sampleTime baseCtrl.value
                local velocity = (valueAtKey - valueBefore) / sampleDt
                velocity * (dampingMultiplier amp freq decay dt matchVelocity)
            )
        )
    ),

    fn springScaleLayer baseCtrl amp freq decay sampleFrames matchVelocity =
    (
        [1,1,1] + (springPoint3Offset baseCtrl amp freq decay sampleFrames matchVelocity)
    ),

    fn springRotationOffset baseCtrl amp freq decay sampleFrames matchVelocity =
    (
        local t = currentTime
        local identityQuat = quat 0 0 0 1


        local keyTime = findPreviousKeyTime baseCtrl t
        if keyTime == undefined then
        (
            identityQuat
        )
        else
        (
            local dt = secondsFromTimeValue (t - keyTime)
            local sampleTime = keyTime - sampleFrames
            local sampleDt = secondsFromTimeValue (keyTime - sampleTime)

            if sampleDt <= 0.0 then
            (
                identityQuat
            )
            else
            (
                local valueAtKey = normalize (at time keyTime baseCtrl.value)
                local valueBefore = normalize (at time sampleTime baseCtrl.value)
                local deltaQuat = normalize (valueAtKey * (inverse valueBefore))
                local deltaAA = deltaQuat as angleaxis
                if (abs deltaAA.angle) < 0.0001 or (length deltaAA.axis) < 0.0001 then
                (
                    identityQuat
                )
                else
                (
                    local angleVelocity = deltaAA.angle / sampleDt
                    local springAngle = angleVelocity * (dampingMultiplier amp freq decay dt matchVelocity)
                    local springQuat = (angleaxis springAngle deltaAA.axis) as quat
                    normalize springQuat
                )
            )
        )
    ),

    fn springRotationOffsetFromNode sourceNode amp freq decay sampleFrames matchVelocity =
    (
        local t = currentTime
        local identityQuat = quat 0 0 0 1

        logRotationHeader sourceNode

        if sourceNode == undefined then
        (
            logRotationDetails "offset reason" "sourceNode undefined"
            identityQuat
        )
        else
        (
            local baseCtrl = sourceNode.rotation.controller
            local keyTime = findPreviousKeyTime baseCtrl t
            logRotationDetails "base controller" (classOf baseCtrl)
            logRotationDetails "keyTime" keyTime

            if keyTime == undefined then
            (
                logRotationDetails "offset reason" "keyTime undefined"
                identityQuat
            )
            else
            (
                local dt = secondsFromTimeValue (t - keyTime)
                local sampleTime = keyTime - sampleFrames
                local sampleDt = secondsFromTimeValue (keyTime - sampleTime)
                logRotationDetails "dt seconds" dt
                logRotationDetails "sampleTime" sampleTime
                logRotationDetails "sampleDt seconds" sampleDt

                if sampleDt <= 0.0 then
                (
                    logRotationDetails "offset reason" "sampleDt <= 0"
                    identityQuat
                )
                else
                (
                    local valueAtKey = normalize (at time keyTime sourceNode.rotation)
                    local valueBefore = normalize (at time sampleTime sourceNode.rotation)
                    local deltaQuat = normalize (valueAtKey * (inverse valueBefore))
                    local deltaAA = deltaQuat as angleaxis
                    logRotationDetails "valueAtKey/sourceNode.rotation" valueAtKey
                    logRotationDetails "valueBefore/sourceNode.rotation" valueBefore
                    logRotationDetails "deltaQuat" deltaQuat
                    logRotationDetails "deltaAA" deltaAA

                    if (abs deltaAA.angle) < 0.0001 or (length deltaAA.axis) < 0.0001 then
                    (
                        logRotationDetails "offset reason" "delta angle/axis too small"
                        identityQuat
                    )
                    else
                    (
                        local angleVelocity = deltaAA.angle / sampleDt
                        local springAngle = angleVelocity * (dampingMultiplier amp freq decay dt matchVelocity)
                        local resultOffset = normalize ((angleaxis springAngle deltaAA.axis) as quat)
                        logRotationDetails "angleVelocity" angleVelocity
                        logRotationDetails "springAngle" springAngle
                        logRotationDetails "resultOffset" resultOffset
                        resultOffset
                    )
                )
            )
        )
    ),

    fn getEulerAxisValues ctrl sampleTime =
    (
        local values = undefined

        try
        (
            if ctrl.numSubs >= 3 do
            (
                local xCtrl = ctrl[1].controller
                local yCtrl = ctrl[2].controller
                local zCtrl = ctrl[3].controller

                if xCtrl != undefined and yCtrl != undefined and zCtrl != undefined do
                (
                    values = [at time sampleTime xCtrl.value, at time sampleTime yCtrl.value, at time sampleTime zCtrl.value]
                )
            )
        )
        catch()

        values
    ),

    fn springEulerRotationOffset baseCtrl amp freq decay sampleFrames matchVelocity =
    (
        local t = currentTime
        local identityQuat = quat 0 0 0 1
        local keyTime = findPreviousKeyTime baseCtrl t

        if keyTime == undefined then
        (
            identityQuat
        )
        else
        (
            local dt = secondsFromTimeValue (t - keyTime)
            local sampleTime = keyTime - sampleFrames
            local sampleDt = secondsFromTimeValue (keyTime - sampleTime)
            local keyEuler = getEulerAxisValues baseCtrl keyTime
            local beforeEuler = getEulerAxisValues baseCtrl sampleTime

            if sampleDt <= 0.0 or keyEuler == undefined or beforeEuler == undefined then
            (
                undefined
            )
            else
            (
                local angularVelocity = (keyEuler - beforeEuler) / sampleDt
                local eulerOffset = angularVelocity * (dampingMultiplier amp freq decay dt matchVelocity)
                eulerAngles eulerOffset.x eulerOffset.y eulerOffset.z as quat
            )
        )
    ),

    fn springEulerOffsetVector baseCtrl amp freq decay sampleFrames matchVelocity =
    (
        local t = currentTime
        local keyTime = findPreviousKeyTime baseCtrl t

        if keyTime == undefined then
        (
            [0,0,0]
        )
        else
        (
            local dt = secondsFromTimeValue (t - keyTime)
            local sampleTime = keyTime - sampleFrames
            local sampleDt = secondsFromTimeValue (keyTime - sampleTime)
            local keyEuler = getEulerAxisValues baseCtrl keyTime
            local beforeEuler = getEulerAxisValues baseCtrl sampleTime

            if sampleDt <= 0.0 or keyEuler == undefined or beforeEuler == undefined then
            (
                [0,0,0]
            )
            else
            (
                local angularVelocity = (keyEuler - beforeEuler) / sampleDt
                angularVelocity * (dampingMultiplier amp freq decay dt matchVelocity)
            )
        )
    ),

    fn springEulerOffsetVectorAt baseCtrl evalTime amp freq decay sampleFrames matchVelocity =
    (
        local keyTime = findLastKeyTime baseCtrl

        if keyTime == undefined then
        (
            [0,0,0]
        )
        else if evalTime <= keyTime then
        (
            [0,0,0]
        )
        else
        (
            local dt = secondsFromTimeValue (evalTime - keyTime)
            local sampleTime = keyTime - sampleFrames
            local sampleDt = secondsFromTimeValue (keyTime - sampleTime)
            local keyEuler = getEulerAxisValues baseCtrl keyTime
            local beforeEuler = getEulerAxisValues baseCtrl sampleTime

            if sampleDt <= 0.0 or keyEuler == undefined or beforeEuler == undefined then
            (
                [0,0,0]
            )
            else
            (
                local angularVelocity = (keyEuler - beforeEuler) / sampleDt
                angularVelocity * (dampingMultiplier amp freq decay dt matchVelocity)
            )
        )
    ),

    fn springEulerAxisFromNode sourceNode axisIndex amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            0.0
        )
        else
        (
            local baseCtrl = sourceNode.rotation.controller
            local baseEuler = getEulerAxisValues baseCtrl currentTime
            local offsetEuler = springEulerOffsetVector baseCtrl amp freq decay sampleFrames matchVelocity

            if baseEuler == undefined then
            (
                0.0
            )
            else
            (
                degreesToRadians ((baseEuler[axisIndex]) + (offsetEuler[axisIndex]))
            )
        )
    ),

    fn springEulerVectorFromNode sourceNode amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            [0,0,0]
        )
        else
        (
            local baseCtrl = sourceNode.rotation.controller
            local baseEuler = getEulerAxisValues baseCtrl currentTime
            local offsetEuler = springEulerOffsetVector baseCtrl amp freq decay sampleFrames matchVelocity

            if baseEuler == undefined then
            (
                [0,0,0]
            )
            else
            (
                baseEuler + offsetEuler
            )
        )
    ),

    fn springPositionFromNodeAt sourceNode evalTime amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            [0,0,0]
        )
        else
        (
            local baseCtrl = sourceNode.pos.controller
            local baseValue = at time evalTime baseCtrl.value
            baseValue + (springPoint3OffsetAt baseCtrl evalTime amp freq decay sampleFrames matchVelocity)
        )
    ),

    fn springEulerVectorFromNodeAt sourceNode evalTime amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            [0,0,0]
        )
        else
        (
            local baseCtrl = sourceNode.rotation.controller
            local baseEuler = getEulerAxisValues baseCtrl evalTime
            local offsetEuler = springEulerOffsetVectorAt baseCtrl evalTime amp freq decay sampleFrames matchVelocity

            if baseEuler == undefined then
            (
                [0,0,0]
            )
            else
            (
                baseEuler + offsetEuler
            )
        )
    ),

    fn springScaleFromNodeAt sourceNode evalTime amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            [1,1,1]
        )
        else
        (
            local baseCtrl = sourceNode.scale.controller
            local baseValue = at time evalTime baseCtrl.value
            baseValue + (springPoint3OffsetAt baseCtrl evalTime amp freq decay sampleFrames matchVelocity)
        )
    ),

    fn springPositionFromNode sourceNode amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            [0,0,0]
        )
        else
        (
            local baseCtrl = sourceNode.pos.controller
            local baseValue = at time currentTime baseCtrl.value
            baseValue + (springPoint3Offset baseCtrl amp freq decay sampleFrames matchVelocity)
        )
    ),

    fn springRotationFromNode sourceNode amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            quat 0 0 0 1
        )
        else
        (
            local baseValue = normalize (at time currentTime sourceNode.rotation)
            local offset = springRotationOffsetFromNode sourceNode amp freq decay sampleFrames matchVelocity
            local result = normalize (offset * baseValue)
            logRotationDetails "baseValue/current sourceNode.rotation" baseValue
            logRotationDetails "final rotation result" result
            result
        )
    ),

    fn springScaleFromNode sourceNode amp freq decay sampleFrames matchVelocity =
    (
        if sourceNode == undefined then
        (
            [1,1,1]
        )
        else
        (
            local baseCtrl = sourceNode.scale.controller
            local baseValue = at time currentTime baseCtrl.value
            baseValue + (springPoint3Offset baseCtrl amp freq decay sampleFrames matchVelocity)
        )
    )
)

SpringKey = SpringKeyCore()

plugin PositionController SpringKey_Position
name:"SpringKey Position"
usePBValidity:false
classID:#(0x7a4c51b1, 0x1d2f4d18)
(
    parameters main rollout:params
    (
        valueController type:#maxObject subAnim:true
        sourceNode type:#node
        amplitude type:#float default:100.0 ui:spnAmplitude
        frequency type:#float default:90.0 ui:spnFrequency
        decay type:#float default:50.0 ui:spnDecay
        sampleFrames type:#float default:0.1 ui:spnSampleFrames
        matchVelocity type:#boolean default:true ui:chkMatchVelocity
    )

    rollout params "SpringKey Position"
    (
        spinner spnAmplitude "Amplitude" range:[-10000.0,10000.0,100.0] type:#float fieldwidth:64
        spinner spnFrequency "Frequency" range:[0.0,10000.0,90.0] type:#float fieldwidth:64
        spinner spnDecay "Decay" range:[0.0,10000.0,50.0] type:#float fieldwidth:64
        spinner spnSampleFrames "Velocity sample (frames)" range:[0.001,100.0,0.1] type:#float fieldwidth:64
        checkbox chkMatchVelocity "Match incoming speed" checked:true
    )

    on create do
    (
        valueController = NewDefaultPositionController()
        usesParentTransform = true
        isLeaf = true
    )

    on getValue do
    (
        SpringKey.springPositionFromNode sourceNode amplitude frequency decay sampleFrames matchVelocity
    )

    on setValue val relVal commit do
    (
        SetControllerValue valueController val commit #absolute
    )
)

plugin RotationController SpringKey_Rotation
name:"SpringKey Rotation"
usePBValidity:false
classID:#(0x7a4c51b2, 0x1d2f4d18)
(
    parameters main rollout:params
    (
        valueController type:#maxObject subAnim:true
        sourceNode type:#node
        amplitude type:#float default:100.0 ui:spnAmplitude
        frequency type:#float default:90.0 ui:spnFrequency
        decay type:#float default:50.0 ui:spnDecay
        sampleFrames type:#float default:0.1 ui:spnSampleFrames
        matchVelocity type:#boolean default:true ui:chkMatchVelocity
    )

    rollout params "SpringKey Rotation"
    (
        spinner spnAmplitude "Amplitude" range:[-10000.0,10000.0,100.0] type:#float fieldwidth:64
        spinner spnFrequency "Frequency" range:[0.0,10000.0,90.0] type:#float fieldwidth:64
        spinner spnDecay "Decay" range:[0.0,10000.0,50.0] type:#float fieldwidth:64
        spinner spnSampleFrames "Velocity sample (frames)" range:[0.001,100.0,0.1] type:#float fieldwidth:64
        checkbox chkMatchVelocity "Match incoming speed" checked:true
    )

    on create do
    (
        valueController = NewDefaultRotationController()
        usesParentTransform = true
        isLeaf = true
    )

    on getValue do
    (
        SpringKey.springRotationFromNode sourceNode amplitude frequency decay sampleFrames matchVelocity
    )

    on setValue val relVal commit do
    (
        SetControllerValue valueController val commit #absolute
    )
)

plugin FloatController SpringKey_RotationAxis
name:"SpringKey Rotation Axis"
usePBValidity:false
classID:#(0x7a4c51b4, 0x1d2f4d18)
(
    parameters main rollout:params
    (
        sourceNode type:#node
        axisIndex type:#integer default:1
        amplitude type:#float default:100.0 ui:spnAmplitude
        frequency type:#float default:90.0 ui:spnFrequency
        decay type:#float default:50.0 ui:spnDecay
        sampleFrames type:#float default:0.1 ui:spnSampleFrames
        matchVelocity type:#boolean default:true ui:chkMatchVelocity
    )

    rollout params "SpringKey Rotation Axis"
    (
        spinner spnAmplitude "Amplitude" range:[-10000.0,10000.0,100.0] type:#float fieldwidth:64
        spinner spnFrequency "Frequency" range:[0.0,10000.0,90.0] type:#float fieldwidth:64
        spinner spnDecay "Decay" range:[0.0,10000.0,50.0] type:#float fieldwidth:64
        spinner spnSampleFrames "Velocity sample (frames)" range:[0.001,100.0,0.1] type:#float fieldwidth:64
        checkbox chkMatchVelocity "Match incoming speed" checked:true
    )

    on getValue do
    (
        SpringKey.springEulerAxisFromNode sourceNode axisIndex amplitude frequency decay sampleFrames matchVelocity
    )

    on setValue val relVal commit do
    (
        val
    )
)

plugin ScaleController SpringKey_Scale
name:"SpringKey Scale"
usePBValidity:false
classID:#(0x7a4c51b3, 0x1d2f4d18)
(
    parameters main rollout:params
    (
        valueController type:#maxObject subAnim:true
        sourceNode type:#node
        amplitude type:#float default:100.0 ui:spnAmplitude
        frequency type:#float default:90.0 ui:spnFrequency
        decay type:#float default:50.0 ui:spnDecay
        sampleFrames type:#float default:0.1 ui:spnSampleFrames
        matchVelocity type:#boolean default:true ui:chkMatchVelocity
    )

    rollout params "SpringKey Scale"
    (
        spinner spnAmplitude "Amplitude" range:[-10000.0,10000.0,100.0] type:#float fieldwidth:64
        spinner spnFrequency "Frequency" range:[0.0,10000.0,90.0] type:#float fieldwidth:64
        spinner spnDecay "Decay" range:[0.0,10000.0,50.0] type:#float fieldwidth:64
        spinner spnSampleFrames "Velocity sample (frames)" range:[0.001,100.0,0.1] type:#float fieldwidth:64
        checkbox chkMatchVelocity "Match incoming speed" checked:true
    )

    on create do
    (
        valueController = NewDefaultScaleController()
        usesParentTransform = true
        isLeaf = true
    )

    on getValue do
    (
        SpringKey.springScaleFromNode sourceNode amplitude frequency decay sampleFrames matchVelocity
    )

    on setValue val relVal commit do
    (
        SetControllerValue valueController val commit #absolute
    )
)

struct SpringKeyTool
(
    fn countControllerKeysIn c totalSoFar =
    (
        local total = totalSoFar

        try
        (
            local keyCount = numKeys c
            if keyCount > 0 do total += keyCount
        )
        catch()

        try
        (
            for i = 1 to c.numSubs do
            (
                try
                (
                    local subAnim = c[i]
                    try(total = countControllerKeysIn subAnim total)catch()
                    if subAnim.controller != undefined do total = countControllerKeysIn subAnim.controller total
                )
                catch()
            )
        )
        catch()

        total
    ),

    fn countControllerKeys ctrl =
    (
        try(countControllerKeysIn ctrl 0)catch(0)
    ),

    fn debugTrack label ctrl =
    (
        local baseCtrl = ctrl
        if (isProperty ctrl #sourceNode and ctrl.sourceNode != undefined) then
        (
            case label of
            (
                "Position": baseCtrl = ctrl.sourceNode.pos.controller
                "Rotation": baseCtrl = ctrl.sourceNode.rotation.controller
                "Scale": baseCtrl = ctrl.sourceNode.scale.controller
            )
        )
        else
        (
            local axisCtrl = undefined
            try(axisCtrl = ctrl[1].controller)catch()
            if classOf axisCtrl == SpringKey_RotationAxis and axisCtrl.sourceNode != undefined then
            (
                baseCtrl = axisCtrl.sourceNode.rotation.controller
            )
            else
            (
                if isProperty ctrl #valueController do baseCtrl = ctrl.valueController
            )
        )

        local keyTime = SpringKey.findPreviousKeyTime baseCtrl currentTime
        local keyCount = countControllerKeys baseCtrl
        format "SpringKey Debug - %\n" label
        format "  controller: %\n" (classOf ctrl)
        format "  base controller: %\n" (classOf baseCtrl)
        if isProperty ctrl #sourceNode do format "  source node: %\n" ctrl.sourceNode
        format "  recursive key count: %\n" keyCount
        format "  current time: % ticks / frame %\n" (currentTime as integer) ((currentTime as float) / TicksPerFrame)
        format "  previous key time: %\n" keyTime
        try(format "  value now: %\n" (at time currentTime baseCtrl.value))catch(format "  value now: <failed>\n")
        if label == "Rotation" do
        (
            local debugSource = undefined
            if isProperty ctrl #sourceNode do debugSource = ctrl.sourceNode
            if debugSource == undefined do
            (
                local axisCtrl = undefined
                try(axisCtrl = ctrl[1].controller)catch()
                if classOf axisCtrl == SpringKey_RotationAxis do debugSource = axisCtrl.sourceNode
            )

            if debugSource != undefined do
            (
                try(format "  source node rotation: %\n" (at time currentTime debugSource.rotation))catch()
                try(format "  source euler axes: %\n" (SpringKey.getEulerAxisValues debugSource.rotation.controller currentTime))catch()
            )
        )
    ),

    fn debugNode node =
    (
        format "\n--- SpringKey Debug Node: % ---\n" node.name
        debugTrack "Position" node.pos.controller
        debugTrack "Rotation" node.rotation.controller
        debugTrack "Scale" node.scale.controller
        try(format "Object rotation property now: %\n" (at time currentTime node.rotation))catch()
        try(format "Object euler axes now: %\n" (SpringKey.getEulerAxisValues node.rotation.controller currentTime))catch()
        format "--- End SpringKey Debug ---\n\n"
    ),

    fn setParams ctrl amp freq decay sample matchVelocity =
    (
        try(ctrl.amplitude = amp)catch()
        try(ctrl.frequency = freq)catch()
        try(ctrl.decay = decay)catch()
        try(ctrl.sampleFrames = sample)catch()
        try(ctrl.matchVelocity = matchVelocity)catch()
    ),

    fn getRotationAxisController rotCtrl axisIndex =
    (
        local axisCtrl = undefined
        try(axisCtrl = rotCtrl[axisIndex].controller)catch()
        axisCtrl
    ),

    fn setRotationAxisController rotCtrl axisIndex axisCtrl =
    (
        try(rotCtrl[axisIndex].controller = axisCtrl)catch()
    ),

    fn makeRotationAxis sourceNode axisIndex amp freq decay sample matchVelocity =
    (
        local axisCtrl = SpringKey_RotationAxis()
        axisCtrl.sourceNode = sourceNode
        axisCtrl.axisIndex = axisIndex
        setParams axisCtrl amp freq decay sample matchVelocity
        axisCtrl
    ),

    fn getSourceNode node =
    (
        local sourceName = undefined
        local sourceNode = undefined

        try(sourceName = getUserProp node "SpringKeySource")catch()
        if sourceName != undefined do try(sourceNode = getNodeByName sourceName)catch()

        if sourceNode == undefined do
        (
            sourceNode = point name:(uniqueName (node.name + "_SpringKey_SRC")) size:20 box:false cross:true axistripod:true centermarker:true
            sourceNode.parent = node.parent
            sourceNode.transform = node.transform
            sourceNode.isHidden = true
            try(setUserProp node "SpringKeySource" sourceNode.name)catch()
        )

        sourceNode
    ),

    fn findExistingSourceNode node =
    (
        local sourceName = undefined
        local sourceNode = undefined
        try(sourceName = getUserProp node "SpringKeySource")catch()
        if sourceName != undefined do try(sourceNode = getNodeByName sourceName)catch()
        sourceNode
    ),

    fn addUniqueTime times t =
    (
        local exists = false
        for item in times while not exists do
        (
            if (abs ((item as float) - (t as float))) < 0.5 do exists = true
        )

        if not exists do append times t
        times
    ),

    fn collectControllerKeyTimesIn ctrl times =
    (
        try
        (
            local keyCount = numKeys ctrl
            if keyCount > 0 do for i = 1 to keyCount do
            (
                addUniqueTime times (getKeyTime ctrl i)
            )
        )
        catch()

        try
        (
            for i = 1 to ctrl.numSubs do
            (
                try
                (
                    local subAnim = ctrl[i]
                    try(collectControllerKeyTimesIn subAnim times)catch()
                    if subAnim.controller != undefined do collectControllerKeyTimesIn subAnim.controller times
                )
                catch()
            )
        )
        catch()

        times
    ),

    fn collectControllerKeyTimes ctrl =
    (
        local times = #()
        try(collectControllerKeyTimesIn ctrl times)catch()
        sort times
        times
    ),

    fn collectSelectedKeyTimesIn ctrl times =
    (
        try
        (
            local keyCount = numKeys ctrl
            if keyCount > 0 do for i = 1 to keyCount do
            (
                try
                (
                    if isKeySelected ctrl i do addUniqueTime times (getKeyTime ctrl i)
                )
                catch()
            )
        )
        catch()

        try
        (
            for i = 1 to ctrl.numSubs do
            (
                try
                (
                    local subAnim = ctrl[i]
                    try(collectSelectedKeyTimesIn subAnim times)catch()
                    if subAnim.controller != undefined do collectSelectedKeyTimesIn subAnim.controller times
                )
                catch()
            )
        )
        catch()

        times
    ),

    fn getSelectedKeyRange node doPosition doRotation doScale =
    (
        local times = #()
        if doPosition do try(collectSelectedKeyTimesIn node.pos.controller times)catch()
        if doRotation do try(collectSelectedKeyTimesIn node.rotation.controller times)catch()
        if doScale do try(collectSelectedKeyTimesIn node.scale.controller times)catch()

        sort times
        if times.count >= 2 then
        (
            #(times[1], times[times.count], times.count)
        )
        else
        (
            undefined
        )
    ),

    fn collectBakeJobs nodes doPosition doRotation doScale =
    (
        local jobs = #()
        for node in nodes do
        (
            local selectedRange = getSelectedKeyRange node doPosition doRotation doScale
            if selectedRange != undefined do append jobs #(node, selectedRange)
        )
        jobs
    ),

    fn findNextSourceKeyTime keyTimes keyTime fallbackEnd =
    (
        local nextTime = fallbackEnd
        for t in keyTimes while nextTime == fallbackEnd do
        (
            if t > keyTime do nextTime = t
        )
        nextTime
    ),

    fn addSpringEventTimes times keyTimes keyTime freq decay rangeEnd threshold =
    (
        local f = freq / 30.0
        local d = decay / 10.0
        local segmentEnd = findNextSourceKeyTime keyTimes keyTime rangeEnd
        local maxDt = SpringKey.secondsFromTimeValue (segmentEnd - keyTime)

        if f > 0.0001 and maxDt > 0.0 do
        (
            local decayLimit = maxDt
            if d > 0.0001 do
            (
                local thresholdLimit = (log (1.0 / threshold)) / d
                if thresholdLimit < decayLimit do decayLimit = thresholdLimit
            )

            local halfPeriod = 1.0 / (2.0 * f)
            local n = 0
            while (n * halfPeriod) <= decayLimit do
            (
                local dt = n * halfPeriod
                if dt > 0.0 do addUniqueTime times (keyTime + (dt * frameRate))
                n += 1
            )

            local w = 2.0 * pi * f
            local firstPeak = if d > 0.0001 then ((atan (w / d)) / (360.0 * f)) else (1.0 / (4.0 * f))
            n = 0
            while (firstPeak + (n / (2.0 * f))) <= decayLimit do
            (
                local dt = firstPeak + (n / (2.0 * f))
                if dt > 0.0 do addUniqueTime times (keyTime + (dt * frameRate))
                n += 1
            )
        )

        times
    ),

    fn getSpringSettleTimes keyTime freq decay rangeEnd threshold =
    (
        local f = freq / 30.0
        local d = decay / 10.0
        local maxDt = SpringKey.secondsFromTimeValue (rangeEnd - keyTime)

        if f <= 0.0001 or maxDt <= 0.0 then
        (
            undefined
        )
        else
        (
            local settleDt = maxDt
            if d > 0.0001 do
            (
                local thresholdLimit = (log (1.0 / threshold)) / d
                if thresholdLimit < settleDt do settleDt = thresholdLimit
            )

            local fadeDuration = 1.0 / (2.0 * f)
            if fadeDuration > settleDt do fadeDuration = settleDt
            local fadeStartDt = settleDt - fadeDuration
            if fadeStartDt < 0.0 do fadeStartDt = 0.0
            #(keyTime + (fadeStartDt * frameRate), keyTime + (settleDt * frameRate))
        )
    ),

    fn buildSparseBakeTimes controllers freq decay threshold =
    (
        local times = #()
        local allSourceKeys = #()
        local rangeStart = animationRange.start
        local rangeEnd = animationRange.end
        local lastSourceKey = undefined

        addUniqueTime times rangeStart

        for ctrl in controllers where ctrl != undefined do
        (
            local keyTimes = collectControllerKeyTimes ctrl
            for keyTime in keyTimes do
            (
                addUniqueTime times keyTime
                addUniqueTime allSourceKeys keyTime
                if lastSourceKey == undefined or keyTime > lastSourceKey do lastSourceKey = keyTime
            )
        )

        sort allSourceKeys
        if lastSourceKey != undefined do addSpringEventTimes times #(lastSourceKey) lastSourceKey freq decay rangeEnd threshold

        sort times
        times
    ),

    fn buildRangeBakeTimes startTime endTime freq decay threshold =
    (
        local times = #()
        SpringKey.logText ("Bake range start: " + (startTime as string) + ", end: " + (endTime as string) + ", animation end: " + (animationRange.end as string))
        addSpringEventTimes times #(endTime) endTime freq decay animationRange.end threshold
        local settleTimes = getSpringSettleTimes endTime freq decay animationRange.end threshold
        if settleTimes != undefined do
        (
            addUniqueTime times settleTimes[1]
            addUniqueTime times settleTimes[2]
        )
        sort times
        times
    ),

    fn frameNumberToTime frameNumber =
    (
        frameNumber as time
    ),

    fn quantizeTimeToStep t stepFrames =
    (
        if stepFrames <= 0.0 then
        (
            t
        )
        else
        (
            local frameValue = (t as float) / TicksPerFrame
            local quantizedFrame = (floor ((frameValue / stepFrames) + 0.5)) * stepFrames
            quantizedFrame as time
        )
    ),

    fn quantizeTimes times stepFrames =
    (
        local result = #()
        for t in times do addUniqueTime result (quantizeTimeToStep t stepFrames)
        sort result
        result
    ),

    fn keepTimesAfter times cutoffTime =
    (
        local result = #()
        for t in times do
        (
            if ((t as float) - (cutoffTime as float)) > 0.5 do addUniqueTime result t
        )
        sort result
        result
    ),

    fn getControllerKey ctrl keyIndex =
    (
        local result = undefined
        try(result = ctrl.keys[keyIndex])catch()
        if result == undefined do try(result = getKey ctrl keyIndex)catch()
        result
    ),

    fn setGeneratedTangentsIn ctrl impactTime tangentType =
    (
        try
        (
            local keyCount = numKeys ctrl
            if keyCount > 0 do for i = 1 to keyCount do
            (
                local keyTime = getKeyTime ctrl i
                if keyTime > impactTime do
                (
                    local key = getControllerKey ctrl i
                    if key != undefined do
                    (
                        try(key.inTangentType = tangentType)catch()
                        try(key.outTangentType = tangentType)catch()
                    )
                )
            )
        )
        catch()

        try
        (
            for i = 1 to ctrl.numSubs do
            (
                try
                (
                    local subAnim = ctrl[i]
                    try(setGeneratedTangentsIn subAnim impactTime tangentType)catch()
                    if subAnim.controller != undefined do setGeneratedTangentsIn subAnim.controller impactTime tangentType
                )
                catch()
            )
        )
        catch()
    ),

    fn setGeneratedTangents node doPosition doRotation doScale impactTime tangentType =
    (
        if doPosition do try(setGeneratedTangentsIn node.pos.controller impactTime tangentType)catch()
        if doRotation do
        (
            local rotCtrl = node.rotation.controller
            try(setGeneratedTangentsIn rotCtrl impactTime tangentType)catch()
            try(setGeneratedTangentsIn rotCtrl[1].controller impactTime tangentType)catch()
            try(setGeneratedTangentsIn rotCtrl[2].controller impactTime tangentType)catch()
            try(setGeneratedTangentsIn rotCtrl[3].controller impactTime tangentType)catch()
        )
        if doScale do try(setGeneratedTangentsIn node.scale.controller impactTime tangentType)catch()
    ),

    fn tailFadeMultiplier evalTime fadeStartTime settleTime =
    (
        if fadeStartTime == undefined or settleTime == undefined then
        (
            1.0
        )
        else if evalTime >= settleTime then
        (
            0.0
        )
        else if evalTime <= fadeStartTime then
        (
            1.0
        )
        else
        (
            local span = (settleTime as float) - (fadeStartTime as float)
            if span <= 0.0 then
            (
                0.0
            )
            else
            (
                local p = ((evalTime as float) - (fadeStartTime as float)) / span
                1.0 - ((p * p) * (3.0 - (2.0 * p)))
            )
        )
    ),

    fn springPoint3FromRangeAt baseCtrl evalTime impactTime amp freq decay sampleFrames matchVelocity bounceMode fadeStartTime settleTime =
    (
        local baseTime = if evalTime <= impactTime then evalTime else impactTime
        local baseValue = at time baseTime baseCtrl.value

        if evalTime <= impactTime then
        (
            baseValue
        )
        else
        (
            local dt = SpringKey.secondsFromTimeValue (evalTime - impactTime)
            local sampleTime = impactTime - sampleFrames
            local sampleDt = SpringKey.secondsFromTimeValue (impactTime - sampleTime)

            if sampleDt <= 0.0 then
            (
                baseValue
            )
            else
            (
                local valueAtKey = at time impactTime baseCtrl.value
                local valueBefore = at time sampleTime baseCtrl.value
                local velocity = (valueAtKey - valueBefore) / sampleDt
                local fade = tailFadeMultiplier evalTime fadeStartTime settleTime
                baseValue + ((velocity * (SpringKey.elasticMultiplier amp freq decay dt matchVelocity bounceMode)) * fade)
            )
        )
    ),

    fn springEulerFromRangeAt baseCtrl evalTime impactTime amp freq decay sampleFrames matchVelocity bounceMode fadeStartTime settleTime =
    (
        local baseTime = if evalTime <= impactTime then evalTime else impactTime
        local baseEuler = SpringKey.getEulerAxisValues baseCtrl baseTime

        if baseEuler == undefined then
        (
            [0,0,0]
        )
        else if evalTime <= impactTime then
        (
            baseEuler
        )
        else
        (
            local dt = SpringKey.secondsFromTimeValue (evalTime - impactTime)
            local sampleTime = impactTime - sampleFrames
            local sampleDt = SpringKey.secondsFromTimeValue (impactTime - sampleTime)
            local keyEuler = SpringKey.getEulerAxisValues baseCtrl impactTime
            local beforeEuler = SpringKey.getEulerAxisValues baseCtrl sampleTime

            if sampleDt <= 0.0 or keyEuler == undefined or beforeEuler == undefined then
            (
                baseEuler
            )
            else
            (
                local angularVelocity = (keyEuler - beforeEuler) / sampleDt
                local fade = tailFadeMultiplier evalTime fadeStartTime settleTime
                baseEuler + ((angularVelocity * (SpringKey.elasticMultiplier amp freq decay dt matchVelocity bounceMode)) * fade)
            )
        )
    ),

    fn bakePositionTrack node sourceNode times impactTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime =
    (
        local values = #()
        for t in times do append values #(t, springPoint3FromRangeAt sourceNode.pos.controller t impactTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime)
        local posCtrl = node.pos.controller

        animate on
        (
            for item in values do at time item[1]
            (
                local value = item[2]
                SpringKey.logText ("Bake Local Position " + (item[1] as string) + " = " + (value as string))
                posCtrl.value = value
            )
        )
    ),

    fn deleteKeysAfterTimeIn ctrl cutoffTime =
    (
        try
        (
            local keyCount = numKeys ctrl
            if keyCount > 0 do for i = keyCount to 1 by -1 do
            (
                try
                (
                    if (getKeyTime ctrl i) > cutoffTime do deleteKey ctrl i
                )
                catch()
            )
        )
        catch()

        try
        (
            for i = 1 to ctrl.numSubs do
            (
                try
                (
                    local subAnim = ctrl[i]
                    try(deleteKeysAfterTimeIn subAnim cutoffTime)catch()
                    if subAnim.controller != undefined do deleteKeysAfterTimeIn subAnim.controller cutoffTime
                )
                catch()
            )
        )
        catch()
    ),

    fn deleteBakedKeysAfterTime node doPosition doRotation doScale cutoffTime =
    (
        if doPosition do try(deleteKeysAfterTimeIn node.pos.controller cutoffTime)catch()
        if doRotation do try(deleteKeysAfterTimeIn node.rotation.controller cutoffTime)catch()
        if doScale do try(deleteKeysAfterTimeIn node.scale.controller cutoffTime)catch()
    ),

    fn bakeRotationTrack node sourceNode times impactTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime =
    (
        local rotCtrl = node.rotation.controller
        if ((classOf rotCtrl as string) != "Euler_XYZ") then
        (
            messageBox "Rotation bake currently requires an Euler_XYZ rotation controller to preserve 360+ spin." title:"SpringKey"
        )
        else
        (
            local values = #()
            for t in times do append values #(t, springEulerFromRangeAt sourceNode.rotation.controller t impactTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime)

            animate on
            (
                for item in values do at time item[1]
                (
                    local value = item[2]
                    SpringKey.logText ("Bake Rotation " + (item[1] as string) + " = " + (value as string))
                    try(rotCtrl[1].controller.value = value.x)catch()
                    try(rotCtrl[2].controller.value = value.y)catch()
                    try(rotCtrl[3].controller.value = value.z)catch()
                )
            )
        )
    ),

    fn bakeScaleTrack node sourceNode times impactTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime =
    (
        local values = #()
        for t in times do append values #(t, springPoint3FromRangeAt sourceNode.scale.controller t impactTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime)
        local scaleCtrl = node.scale.controller

        animate on
        (
            for item in values do at time item[1]
            (
                local value = item[2]
                SpringKey.logText ("Bake Local Scale " + (item[1] as string) + " = " + (value as string))
                scaleCtrl.value = value
            )
        )
    ),

    fn bakeNodeRange node doPosition doRotation doScale startTime endTime amp freq decay sample matchVelocity bounceMode threshold keyStep =
    (
        local sourceNode = point name:(uniqueName (node.name + "_SpringKey_BAKE_SRC")) size:20 box:false cross:true axistripod:true centermarker:true
        sourceNode.parent = node.parent
        sourceNode.transform = node.transform
        sourceNode.isHidden = true

        if doPosition do try(sourceNode.pos.controller = copy node.pos.controller)catch(sourceNode.pos.controller = node.pos.controller)
        if doRotation do try(sourceNode.rotation.controller = copy node.rotation.controller)catch(sourceNode.rotation.controller = node.rotation.controller)
        if doScale do try(sourceNode.scale.controller = copy node.scale.controller)catch(sourceNode.scale.controller = node.scale.controller)

        if doPosition do try(deleteKeysAfterTimeIn sourceNode.pos.controller endTime)catch()
        if doRotation do try(deleteKeysAfterTimeIn sourceNode.rotation.controller endTime)catch()
        if doScale do try(deleteKeysAfterTimeIn sourceNode.scale.controller endTime)catch()
        deleteBakedKeysAfterTime node doPosition doRotation doScale endTime

        local settleTimes = getSpringSettleTimes endTime freq decay animationRange.end threshold
        local fadeStartTime = if settleTimes != undefined then settleTimes[1] else undefined
        local settleTime = if settleTimes != undefined then quantizeTimeToStep settleTimes[2] keyStep else undefined
        local times = buildRangeBakeTimes startTime endTime freq decay threshold
        SpringKey.logText ("Bake raw key count: " + (times.count as string) + ", key step: " + (keyStep as string))
        times = quantizeTimes times keyStep
        if settleTime != undefined and ((settleTime as float) - (endTime as float)) > 0.5 do addUniqueTime times settleTime
        times = keepTimesAfter times endTime
        sort times
        if settleTime != undefined do SpringKey.logText ("Bake settle key: " + (settleTime as string))
        SpringKey.logText ("Bake node: " + node.name + ", key count: " + (times.count as string))
        if times.count > 0 do
        (
            SpringKey.logText ("Bake first time: " + (times[1] as string))
            SpringKey.logText ("Bake last time: " + (times[times.count] as string))
        )

        if doPosition do bakePositionTrack node sourceNode times endTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime
        if doRotation do bakeRotationTrack node sourceNode times endTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime
        if doScale do bakeScaleTrack node sourceNode times endTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime
        setGeneratedTangents node doPosition doRotation doScale endTime (if bounceMode then #fast else #slow)

        try(delete sourceNode)catch()
    ),

    fn bakeNode node doPosition doRotation doScale amp freq decay sample matchVelocity threshold =
    (
        local startTime = animationRange.start
        local endTime = trackbar.getPreviousKeyTime()
        if endTime == undefined do endTime = animationRange.end
        bakeNodeRange node doPosition doRotation doScale startTime endTime amp freq decay sample matchVelocity false threshold 1.0
    ),

    fn wrapPosition node amp freq decay sample matchVelocity =
    (
        local ctrl = node.pos.controller
        local sourceNode = getSourceNode node

        if (classOf ctrl != SpringKey_Position) then
        (
            sourceNode.pos.controller = ctrl
            ctrl = SpringKey_Position()
            ctrl.usesParentTransform = true
            ctrl.sourceNode = sourceNode
            node.pos.controller = ctrl
        )
        else
        (
            ctrl.sourceNode = sourceNode
        )

        setParams ctrl amp freq decay sample matchVelocity
    ),

    fn wrapRotation node amp freq decay sample matchVelocity =
    (
        local ctrl = node.rotation.controller
        local sourceNode = getSourceNode node

        local existingAxis = undefined
        try(existingAxis = getRotationAxisController ctrl 1)catch()
        if classOf existingAxis == SpringKey_RotationAxis and existingAxis.sourceNode != undefined do
        (
            node.rotation.controller = existingAxis.sourceNode.rotation.controller
            ctrl = node.rotation.controller
        )

        if (classOf ctrl != SpringKey_Rotation) then
        (
            sourceNode.rotation.controller = ctrl
            ctrl = SpringKey_Rotation()
            ctrl.usesParentTransform = true
            ctrl.sourceNode = sourceNode
            node.rotation.controller = ctrl
        )
        else
        (
            ctrl.sourceNode = sourceNode
        )

        setParams ctrl amp freq decay sample matchVelocity
    ),

    fn wrapScale node amp freq decay sample matchVelocity =
    (
        local ctrl = node.scale.controller
        local sourceNode = getSourceNode node

        if (classOf ctrl != SpringKey_Scale) then
        (
            sourceNode.scale.controller = ctrl
            ctrl = SpringKey_Scale()
            ctrl.usesParentTransform = true
            ctrl.sourceNode = sourceNode
            node.scale.controller = ctrl
        )
        else
        (
            ctrl.sourceNode = sourceNode
        )

        setParams ctrl amp freq decay sample matchVelocity
    ),

    fn unwrapPosition node =
    (
        local ctrl = node.pos.controller
        if classOf ctrl == SpringKey_Position and ctrl.sourceNode != undefined do node.pos.controller = ctrl.sourceNode.pos.controller
    ),

    fn unwrapRotation node =
    (
        local ctrl = node.rotation.controller
        if classOf ctrl == SpringKey_Rotation and ctrl.sourceNode != undefined do node.rotation.controller = ctrl.sourceNode.rotation.controller
        if ((classOf ctrl as string) == "Euler_XYZ") do
        (
            local axisCtrl = getRotationAxisController ctrl 1
            if classOf axisCtrl == SpringKey_RotationAxis and axisCtrl.sourceNode != undefined do
            (
                node.rotation.controller = axisCtrl.sourceNode.rotation.controller
            )
        )
    ),

    fn unwrapScale node =
    (
        local ctrl = node.scale.controller
        if classOf ctrl == SpringKey_Scale and ctrl.sourceNode != undefined do node.scale.controller = ctrl.sourceNode.scale.controller
    )
)

global SpringKeyUI = SpringKeyTool()

try(destroyDialog SpringKey_Rollout)catch()

rollout SpringKey_Rollout "SpringKey" width:248
(
    group "Tracks"
    (
        checkbox chkPosition "Position" checked:true across:3 tooltip:"Bake the spring result onto the selected object's Position track."
        checkbox chkRotation "Rotation" checked:true tooltip:"Bake the spring result onto the selected object's Rotation track."
        checkbox chkScale "Scale" checked:false tooltip:"Bake the spring result onto the selected object's Scale track."
    )

    group "Elastic"
    (
        spinner spnAmplitude "Amplitude" range:[-10000.0,10000.0,100.0] type:#float fieldwidth:72 align:#right tooltip:"Controls the spring amount. Higher values create a larger overshoot; negative values invert the response."
        spinner spnFrequency "Frequency" range:[0.0,10000.0,90.0] type:#float fieldwidth:72 align:#right tooltip:"Controls how fast the spring oscillates. Higher values add more waves over the same time."
        spinner spnDecay "Decay" range:[0.0,10000.0,50.0] type:#float fieldwidth:72 align:#right tooltip:"Controls how quickly the spring fades out. Higher values stop the motion sooner."
        spinner spnSampleFrames "Velocity sample (frames)" range:[0.001,100.0,0.1] type:#float fieldwidth:72 align:#right tooltip:"How many frames before the impact key are sampled to estimate incoming velocity. Smaller values react to the immediate tangent; larger values average over a wider section."
        checkbox chkMatchVelocity "Match incoming speed" checked:true align:#left tooltip:"Scale the spring by the incoming velocity so faster animation produces a stronger overshoot. Turn off for a more fixed, amplitude-driven response."
        checkbox chkBounce "Bounce" checked:false align:#left tooltip:"Use a one-sided bounce response opposite the incoming motion. Uses the same amplitude, frequency, decay, and velocity settings."
    )

    group "Sparse Bake"
    (
        spinner spnKeyStep "Key step" range:[0.0,1000.0,1.0] type:#float scale:0.1 fieldwidth:72 align:#right tooltip:"Snap generated bake keys to this frame interval. 1.0 means whole frames; 2.0 means every 2 frames; 0 disables snapping."
        spinner spnBakeThreshold "Stop threshold" range:[0.0001,1.0,0.01] type:#float scale:0.001 fieldwidth:72 align:#right tooltip:"Start ending the spring tail once the damped response falls below this level, then fade to a final base-value settle key. Lower values keep a longer tail."
        button btnBake "Bake Spring/Bounce To Keys" width:214 height:28 tooltip:"Bake the spring or bounce result onto selected nodes. Requires at least two selected keys on the enabled tracks."
        label lblSelectedKeys "Select at least two keys." align:#center tooltip:"SpringKey uses the selected key range as the source interval for the bake."
    )

    button btnDebug "Debug Selected" width:214 height:24 tooltip:"Print diagnostic information for the selected nodes to the MAXScript Listener and to the log when logging is enabled."

    group "Logging"
    (
        checkbox chkEnableLog "Enable Log" checked:false align:#left tooltip:"Write debug and bake details to SpringKey_debug.log next to this script."
        button btnClearLog "Clear Log File" width:214 height:22 tooltip:"Clear SpringKey_debug.log and write a fresh header."
        label lblLogPath "SpringKey_debug.log" align:#center tooltip:"The log file is written next to the loaded SpringKey.ms file."
    )

    label lblInfo "Sparse baked keys only." align:#center tooltip:"This rollout creates baked keys instead of leaving live SpringKey controllers on the object."

    on chkEnableLog changed state do
    (
        SpringKey.setLogging state
        if state do SpringKey.logText "Logging enabled from UI."
    )

    on btnClearLog pressed do
    (
        SpringKey.clearLog()
    )

    on btnBake pressed do
    (
        if selection.count == 0 then
        (
            messageBox "Select one or more nodes first." title:"SpringKey"
        )
        else
        (
            local jobs = SpringKeyUI.collectBakeJobs selection chkPosition.checked chkRotation.checked chkScale.checked
            if jobs.count == 0 then
            (
                SpringKey.logText "Bake skipped: no selected key range on enabled tracks."
                messageBox "Select at least two keys on the enabled tracks before baking." title:"SpringKey"
            )
            else
            (
            local oldTime = currentTime
            local oldAnimate = animateMode
            local setupTime = animationRange.start

            undo "Bake SpringKey" on
            (
                try
                (
                    animateMode = false
                    sliderTime = setupTime
                    at time setupTime
                    (
                        for job in jobs do
                        (
                            local node = job[1]
                            local selectedRange = job[2]
                            local startTime = selectedRange[1]
                            local endTime = selectedRange[2]
                            SpringKey.logText ("Using selected key range on " + node.name + ": " + (startTime as string) + " -> " + (endTime as string) + ", selected times: " + (selectedRange[3] as string))
                            SpringKeyUI.bakeNodeRange node chkPosition.checked chkRotation.checked chkScale.checked startTime endTime spnAmplitude.value spnFrequency.value spnDecay.value spnSampleFrames.value chkMatchVelocity.checked chkBounce.checked spnBakeThreshold.value spnKeyStep.value
                        )
                    )
                )
                catch
                (
                    try(sliderTime = oldTime)catch()
                    try(animateMode = oldAnimate)catch()
                    throw()
                )
            )

            try(sliderTime = oldTime)catch()
            try(animateMode = oldAnimate)catch()
            redrawViews()
            )
        )
    )

    on btnDebug pressed do
    (
        if selection.count == 0 then
        (
            messageBox "Select one or more nodes first." title:"SpringKey"
        )
        else
        (
            for node in selection do
            (
                SpringKey.logText ("Debug node: " + node.name)
                SpringKeyUI.debugNode node
            )
        )
    )
)

createDialog SpringKey_Rollout style:#(#style_titlebar, #style_sysmenu, #style_toolwindow)
