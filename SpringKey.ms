/*
    SpringKey
    Sparse spring/elastic key generator for 3ds Max.

    Run this file, then use the SpringKey rollout to create Position,
    Rotation, and/or Scale spring or bounce keys on selected nodes.
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
        if times.count >= 1 then
        (
            #(times[times.count], times[times.count], times.count)
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
        SpringKey.logText ("Create key range start: " + (startTime as string) + ", end: " + (endTime as string) + ", animation end: " + (animationRange.end as string))
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

    fn buildPeakBakeTimes keyTime freq decay rangeEnd threshold keyStep =
    (
        local times = #()
        local f = freq / 30.0
        local d = decay / 10.0
        local maxDt = SpringKey.secondsFromTimeValue (rangeEnd - keyTime)

        if f > 0.0001 and maxDt > 0.0 do
        (
            local decayLimit = maxDt
            if d > 0.0001 do
            (
                local thresholdLimit = (log (1.0 / threshold)) / d
                if thresholdLimit < decayLimit do decayLimit = thresholdLimit
            )

            local w = 2.0 * pi * f
            local firstPeak = if d > 0.0001 then ((atan (w / d)) / (360.0 * f)) else (1.0 / (4.0 * f))
            local n = 0
            while (firstPeak + (n / (2.0 * f))) <= decayLimit do
            (
                local dt = firstPeak + (n / (2.0 * f))
                if dt > 0.0 do addUniqueTime times (quantizeTimeToStep (keyTime + (dt * frameRate)) keyStep)
                n += 1
            )
        )

        times = keepTimesAfter times keyTime
        sort times
        times
    ),

    fn getControllerKey ctrl keyIndex =
    (
        local result = undefined
        try(result = ctrl.keys[keyIndex])catch()
        if result == undefined do try(result = getKey ctrl keyIndex)catch()
        result
    ),

    fn keyTimeMatchesAny keyTime times toleranceTicks =
    (
        local matches = false
        for t in times while matches == false do
        (
            if (abs ((keyTime as float) - (t as float))) <= toleranceTicks do matches = true
        )
        matches
    ),

    fn setKeyTangentType key tangentType =
    (
        if tangentType == #auto then
        (
            try(key.inTangentType = #auto)catch(try(key.inTangentType = #smooth)catch())
            try(key.outTangentType = #auto)catch(try(key.outTangentType = #smooth)catch())
        )
        else
        (
            try(key.inTangentType = tangentType)catch()
            try(key.outTangentType = tangentType)catch()
        )
    ),

    fn setKeyTangentTypeAtTimeIn ctrl targetTime tangentType =
    (
        try
        (
            local keyCount = numKeys ctrl
            if keyCount > 0 do for i = 1 to keyCount do
            (
                local keyTime = getKeyTime ctrl i
                if (abs ((keyTime as float) - (targetTime as float))) <= 0.5 do
                (
                    local key = getControllerKey ctrl i
                    if key != undefined do setKeyTangentType key tangentType
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
                    try(setKeyTangentTypeAtTimeIn subAnim targetTime tangentType)catch()
                    if subAnim.controller != undefined do setKeyTangentTypeAtTimeIn subAnim.controller targetTime tangentType
                )
                catch()
            )
        )
        catch()
    ),

    fn setKeyTangentTypeAtTime node doPosition doRotation doScale targetTime tangentType =
    (
        if doPosition do try(setKeyTangentTypeAtTimeIn node.pos.controller targetTime tangentType)catch()
        if doRotation do
        (
            local rotCtrl = node.rotation.controller
            try(setKeyTangentTypeAtTimeIn rotCtrl targetTime tangentType)catch()
            try(setKeyTangentTypeAtTimeIn rotCtrl[1].controller targetTime tangentType)catch()
            try(setKeyTangentTypeAtTimeIn rotCtrl[2].controller targetTime tangentType)catch()
            try(setKeyTangentTypeAtTimeIn rotCtrl[3].controller targetTime tangentType)catch()
        )
        if doScale do try(setKeyTangentTypeAtTimeIn node.scale.controller targetTime tangentType)catch()
    ),

    fn scalarComponents value =
    (
        local result = #()
        try(append result (value as float))catch()
        if result.count == 0 do
        (
            try
            (
                append result (value.x as float)
                append result (value.y as float)
                append result (value.z as float)
            )
            catch()
        )
        result
    ),

    fn getAnticipationLength startValue nextValue inTangent segmentFrames =
    (
        local targetLength = undefined
        local startComponents = scalarComponents startValue
        local nextComponents = scalarComponents nextValue
        local tangentComponents = scalarComponents inTangent
        local componentCount = startComponents.count
        if nextComponents.count < componentCount do componentCount = nextComponents.count
        if tangentComponents.count < componentCount do componentCount = tangentComponents.count

        if segmentFrames > 0.0001 and componentCount > 0 do
        (
            for i = 1 to componentCount do
            (
                local slope = tangentComponents[i]
                if (abs slope) > 0.000001 do
                (
                    local lengthValue = (nextComponents[i] - startComponents[i]) / (slope * segmentFrames)
                    if lengthValue > 0.0 do
                    (
                        if targetLength == undefined or lengthValue > targetLength do targetLength = lengthValue
                    )
                )
            )
        )

        targetLength
    ),

    fn findNextKeyIndexAfter ctrl startTime =
    (
        local nextIndex = undefined
        local nextTime = undefined
        try
        (
            local keyCount = numKeys ctrl
            if keyCount > 0 do for i = 1 to keyCount do
            (
                local keyTime = getKeyTime ctrl i
                if ((keyTime as float) - (startTime as float)) > 0.5 do
                (
                    if nextTime == undefined or keyTime < nextTime do
                    (
                        nextTime = keyTime
                        nextIndex = i
                    )
                )
            )
        )
        catch()
        nextIndex
    ),

    fn createAnticipationIn ctrl =
    (
        local changed = 0

        try
        (
            local keyCount = numKeys ctrl
            if keyCount > 0 do for i = 1 to keyCount do
            (
                local startKeySelected = false
                try(startKeySelected = isKeySelected ctrl i)catch()
                if startKeySelected do
                (
                    local startTime = getKeyTime ctrl i
                    local nextIndex = findNextKeyIndexAfter ctrl startTime
                    if nextIndex != undefined do
                    (
                        local startKey = getControllerKey ctrl i
                        local nextKey = getControllerKey ctrl nextIndex
                        local nextTime = getKeyTime ctrl nextIndex
                        local startValue = at time startTime ctrl.value
                        local nextValue = at time nextTime ctrl.value
                        local inTangent = undefined
                        try(inTangent = nextKey.inTangent)catch()

                        if startKey != undefined and nextKey != undefined and inTangent != undefined do
                        (
                            local segmentFrames = ((nextTime as float) - (startTime as float)) / TicksPerFrame
                            local targetLength = getAnticipationLength startValue nextValue inTangent segmentFrames
                            if targetLength != undefined do
                            (
                                setKeyTangentType startKey #auto
                                try(nextKey.freeHandle = true)catch()
                                try(nextKey.x_locked = false)catch()
                                try(nextKey.y_locked = false)catch()
                                try(nextKey.z_locked = false)catch()
                                try(nextKey.inTangentType = #custom)catch()
                                try(nextKey.inTangent = inTangent)catch()
                                try(nextKey.inTangentLength = targetLength)catch()
                                changed += 1
                                SpringKey.logText ("Create anticipation tangent at " + (startTime as string) + " -> " + (nextTime as string) + ", length: " + (targetLength as string))
                            )
                        )
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
                    try(changed += createAnticipationIn subAnim)catch()
                    if subAnim.controller != undefined do changed += createAnticipationIn subAnim.controller
                )
                catch()
            )
        )
        catch()

        changed
    ),

    fn createAnticipation node doPosition doRotation doScale =
    (
        local changed = 0
        if doPosition do try(changed += createAnticipationIn node.pos.controller)catch()
        if doRotation do
        (
            local rotCtrl = node.rotation.controller
            try(changed += createAnticipationIn rotCtrl)catch()
            try(changed += createAnticipationIn rotCtrl[1].controller)catch()
            try(changed += createAnticipationIn rotCtrl[2].controller)catch()
            try(changed += createAnticipationIn rotCtrl[3].controller)catch()
        )
        if doScale do try(changed += createAnticipationIn node.scale.controller)catch()
        changed
    ),

    fn setGeneratedTangentsIn ctrl impactTime peakTimes mainTangentType peakTangentType =
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
                        local tangentType = if keyTimeMatchesAny keyTime peakTimes 0.5 then peakTangentType else mainTangentType
                        setKeyTangentType key tangentType
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
                    try(setGeneratedTangentsIn subAnim impactTime peakTimes mainTangentType peakTangentType)catch()
                    if subAnim.controller != undefined do setGeneratedTangentsIn subAnim.controller impactTime peakTimes mainTangentType peakTangentType
                )
                catch()
            )
        )
        catch()
    ),

    fn setGeneratedTangents node doPosition doRotation doScale impactTime peakTimes mainTangentType peakTangentType =
    (
        if doPosition do try(setGeneratedTangentsIn node.pos.controller impactTime peakTimes mainTangentType peakTangentType)catch()
        if doRotation do
        (
            local rotCtrl = node.rotation.controller
            try(setGeneratedTangentsIn rotCtrl impactTime peakTimes mainTangentType peakTangentType)catch()
            try(setGeneratedTangentsIn rotCtrl[1].controller impactTime peakTimes mainTangentType peakTangentType)catch()
            try(setGeneratedTangentsIn rotCtrl[2].controller impactTime peakTimes mainTangentType peakTangentType)catch()
            try(setGeneratedTangentsIn rotCtrl[3].controller impactTime peakTimes mainTangentType peakTangentType)catch()
        )
        if doScale do try(setGeneratedTangentsIn node.scale.controller impactTime peakTimes mainTangentType peakTangentType)catch()
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
                SpringKey.logText ("Create Local Position key " + (item[1] as string) + " = " + (value as string))
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
            messageBox "Rotation key creation currently requires an Euler_XYZ rotation controller to preserve 360+ spin." title:"SpringKey"
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
                    SpringKey.logText ("Create Rotation key " + (item[1] as string) + " = " + (value as string))
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
                SpringKey.logText ("Create Local Scale key " + (item[1] as string) + " = " + (value as string))
                scaleCtrl.value = value
            )
        )
    ),

    fn bakeNodeRange node doPosition doRotation doScale startTime endTime amp freq decay sample matchVelocity bounceMode threshold keyStep autoSpringImpactTangent =
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
        SpringKey.logText ("Create raw key count: " + (times.count as string) + ", key step: " + (keyStep as string))
        times = quantizeTimes times keyStep
        if settleTime != undefined and ((settleTime as float) - (endTime as float)) > 0.5 do addUniqueTime times settleTime
        times = keepTimesAfter times endTime
        sort times
        if settleTime != undefined do SpringKey.logText ("Create settle key: " + (settleTime as string))
        SpringKey.logText ("Create keys on node: " + node.name + ", key count: " + (times.count as string))
        if times.count > 0 do
        (
            SpringKey.logText ("Create first key time: " + (times[1] as string))
            SpringKey.logText ("Create last key time: " + (times[times.count] as string))
        )

        if doPosition do bakePositionTrack node sourceNode times endTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime
        if doRotation do bakeRotationTrack node sourceNode times endTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime
        if doScale do bakeScaleTrack node sourceNode times endTime amp freq decay sample matchVelocity bounceMode fadeStartTime settleTime
        local peakTimes = buildPeakBakeTimes endTime freq decay animationRange.end threshold keyStep
        local mainTangentType = if bounceMode then #fast else #auto
        setGeneratedTangents node doPosition doRotation doScale endTime peakTimes mainTangentType #slow
        if (autoSpringImpactTangent and (not bounceMode)) do setKeyTangentTypeAtTime node doPosition doRotation doScale endTime #auto

        try(delete sourceNode)catch()
    ),

    fn bakeNode node doPosition doRotation doScale amp freq decay sample matchVelocity threshold =
    (
        local startTime = animationRange.start
        local endTime = trackbar.getPreviousKeyTime()
        if endTime == undefined do endTime = animationRange.end
        bakeNodeRange node doPosition doRotation doScale startTime endTime amp freq decay sample matchVelocity false threshold 1.0 false
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

rollout SpringKey_Rollout "SpringKey" width:500
(
    group "Tracks"
    (
        checkbox chkPosition "Position" checked:true across:3 tooltip:"Create spring or bounce keys on the selected object's Position track."
        checkbox chkRotation "Rotation" checked:true tooltip:"Create spring or bounce keys on the selected object's Rotation track."
        checkbox chkScale "Scale" checked:false tooltip:"Create spring or bounce keys on the selected object's Scale track."
    )

    group "Elastic"
    (
        spinner spnAmplitude "Amplitude" range:[-10000.0,10000.0,100.0] type:#float fieldwidth:72 align:#right tooltip:"Controls the spring amount. Higher values create a larger overshoot; negative values invert the response."
        spinner spnFrequency "Frequency" range:[0.0,10000.0,90.0] type:#float fieldwidth:72 align:#right tooltip:"Controls how fast the spring oscillates. Higher values add more waves over the same time."
        spinner spnDecay "Decay" range:[0.0,10000.0,50.0] type:#float fieldwidth:72 align:#right tooltip:"Controls how quickly the spring fades out. Higher values stop the motion sooner."
        spinner spnSampleFrames "Velocity sample (frames)" range:[0.001,100.0,0.1] type:#float fieldwidth:72 align:#right tooltip:"How many frames before the impact key are sampled to estimate incoming velocity. Smaller values react to the immediate tangent; larger values average over a wider section."
        checkbox chkMatchVelocity "Match incoming speed" checked:true align:#left tooltip:"Scale the spring by the incoming velocity so faster animation produces a stronger overshoot. Turn off for a more fixed, amplitude-driven response."
        checkbox chkBounce "Bounce" checked:false align:#left tooltip:"Use a one-sided bounce response opposite the incoming motion. Uses the same amplitude, frequency, decay, and velocity settings."
        checkbox chkAutoSpringImpact "Auto spring impact tangent" checked:false align:#left tooltip:"Spring only: set the selected impact key to Auto tangents so it behaves like the main generated spring keys. Leave off to preserve the original animation key."
    )

    group "Generated Keys"
    (
        spinner spnKeyStep "Key step" range:[0.0,1000.0,1.0] type:#float scale:0.1 fieldwidth:72 align:#right tooltip:"Snap generated keys to this frame interval. 1.0 means whole frames; 2.0 means every 2 frames; 0 disables snapping."
        spinner spnBakeThreshold "Stop threshold" range:[0.0001,1.0,0.01] type:#float scale:0.001 fieldwidth:72 align:#right tooltip:"Start ending the spring tail once the damped response falls below this level, then fade to a final base-value settle key. Lower values keep a longer tail."
        button btnBake "Create Spring/Bounce Keys" width:214 height:28 tooltip:"Create spring or bounce keys after the last selected key on the enabled tracks."
        label lblSelectedKeys "Select an impact key." align:#center tooltip:"SpringKey uses the last selected key on the enabled tracks as the impact key."
    )

    button btnDebug "Debug Selected" width:214 height:24 tooltip:"Print diagnostic information for the selected nodes to the MAXScript Listener and to the log when logging is enabled."

    group "Logging"
    (
        checkbox chkEnableLog "Enable Log" checked:false align:#left tooltip:"Write debug and key creation details to SpringKey_debug.log next to this script."
        button btnClearLog "Clear Log File" width:214 height:22 tooltip:"Clear SpringKey_debug.log and write a fresh header."
        label lblLogPath "SpringKey_debug.log" align:#center tooltip:"The log file is written next to the loaded SpringKey.ms file."
    )

    label lblInfo "Sparse generated keys only." align:#center tooltip:"This rollout writes animation keys directly instead of leaving live SpringKey controllers on the object."

    group "Anticipation" pos:[258,6] width:226
    (
        button btnAnticipation "Create Anticipation Tangent" width:202 height:28 tooltip:"Adjust the selected start key and the next key. The start key becomes Auto; the next key keeps its incoming tangent direction but extends its incoming handle to meet the start key tangent."
        label lblAnticipation "Select a start key." align:#center tooltip:"Select the first key of a movement. SpringKey uses the next key on the same enabled track as the tangent target."
    )

    on chkEnableLog changed state do
    (
        SpringKey.setLogging state
        if state do SpringKey.logText "Logging enabled from UI."
    )

    on chkBounce changed state do
    (
        if state do chkAutoSpringImpact.checked = false
        chkAutoSpringImpact.enabled = not state
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
                SpringKey.logText "Key creation skipped: no selected impact key on enabled tracks."
                messageBox "Select an impact key on the enabled tracks before creating keys." title:"SpringKey"
            )
            else
            (
            local oldTime = currentTime
            local oldAnimate = animateMode
            local setupTime = animationRange.start

            undo "Create SpringKey Keys" on
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
                            SpringKey.logText ("Using selected impact key on " + node.name + ": " + (endTime as string) + ", selected times: " + (selectedRange[3] as string))
                            local autoSpringImpactTangent = chkAutoSpringImpact.checked and (not chkBounce.checked)
                            SpringKeyUI.bakeNodeRange node chkPosition.checked chkRotation.checked chkScale.checked startTime endTime spnAmplitude.value spnFrequency.value spnDecay.value spnSampleFrames.value chkMatchVelocity.checked chkBounce.checked spnBakeThreshold.value spnKeyStep.value autoSpringImpactTangent
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

    on btnAnticipation pressed do
    (
        if selection.count == 0 then
        (
            messageBox "Select one or more nodes first." title:"SpringKey"
        )
        else
        (
            local changed = 0
            undo "Create Anticipation Tangent" on
            (
                for node in selection do
                (
                    changed += SpringKeyUI.createAnticipation node chkPosition.checked chkRotation.checked chkScale.checked
                )
            )

            if changed == 0 then
            (
                SpringKey.logText "Anticipation skipped: select a start key with a following key on the enabled tracks."
                messageBox "Select a start key with a following key on the enabled tracks." title:"SpringKey"
            )
            else
            (
                SpringKey.logText ("Anticipation tangent changes: " + (changed as string))
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
