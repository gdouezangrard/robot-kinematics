function getObjectName(inInts, _inFloats, _inStrings, _inBuffer)
	n = sim.getObjectName(inInts[1])
	return {}, {}, {n}, ''
end

function getJointType(inInts, _inFloats, _inStrings, _inBuffer)
	t = sim.getJointType(inInts[1])
	return {t}, {}, {}, ''
end

function getJointInterval(inInts, _inFloats, _inStrings, _inBuffer)
	c, i = sim.getJointInterval(inInts[1])
	return {c and 1 or 0}, i, {}, ''
end
