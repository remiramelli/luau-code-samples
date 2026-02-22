local SpringModule = {}
SpringModule.__index = SpringModule

local REST_EPSILON = 0.001
local DAMPING_EPSILON = 0.0001
local MIN_MASS = 0.0001

local function zeroLike(value: any): any
	local valueType = typeof(value)
	if valueType == "Vector3" then
		return Vector3.new(0, 0, 0)
	end
	if valueType == "number" then
		return 0
	end
	return value * 0
end

local function magnitude(value: any): number
	local valueType = typeof(value)
	if valueType == "Vector3" then
		return value.Magnitude
	end
	if valueType == "number" then
		return math.abs(value)
	end
	return 0
end

local function clampVelocity(velocity: any, maxVel: any): any
	if typeof(velocity) == "Vector3" and typeof(maxVel) == "Vector3" then
		local clampedVelocity = velocity
		if maxVel.Magnitude > 0 and clampedVelocity.Magnitude > maxVel.Magnitude then
			clampedVelocity = clampedVelocity.Unit * maxVel.Magnitude
		end
		if math.abs(clampedVelocity.X) > math.abs(maxVel.X) then
			local signX = clampedVelocity.X >= 0 and 1 or -1
			clampedVelocity = Vector3.new(signX * math.abs(maxVel.X), clampedVelocity.Y, clampedVelocity.Z)
		end
		if math.abs(clampedVelocity.Y) > math.abs(maxVel.Y) then
			local signY = clampedVelocity.Y >= 0 and 1 or -1
			clampedVelocity = Vector3.new(clampedVelocity.X, signY * math.abs(maxVel.Y), clampedVelocity.Z)
		end
		if math.abs(clampedVelocity.Z) > math.abs(maxVel.Z) then
			local signZ = clampedVelocity.Z >= 0 and 1 or -1
			clampedVelocity = Vector3.new(clampedVelocity.X, clampedVelocity.Y, signZ * math.abs(maxVel.Z))
		end
		return clampedVelocity
	end

	if typeof(velocity) == "number" and typeof(maxVel) == "number" then
		local velAbs = math.abs(velocity)
		local maxVelAbs = math.abs(maxVel)
		if velAbs > maxVelAbs then
			local sign = velocity >= 0 and 1 or -1
			return sign * maxVelAbs
		end
		return velocity
	end

	return velocity
end

local function solveSpringStep(offset: any, velocity: any, angularFrequency: number, dampingRatio: number, dt: number): (any, any)
	if dampingRatio < (1 - DAMPING_EPSILON) then
		local omegaZeta = angularFrequency * dampingRatio
		local omegaD = angularFrequency * math.sqrt(1 - dampingRatio * dampingRatio)
		local expTerm = math.exp(-omegaZeta * dt)
		local sinTerm = math.sin(omegaD * dt)
		local cosTerm = math.cos(omegaD * dt)
		local c1 = offset
		local c2 = (velocity + offset * omegaZeta) / omegaD
		local newOffset = (c1 * cosTerm + c2 * sinTerm) * expTerm
		local newVelocity = ((c2 * omegaD - c1 * omegaZeta) * cosTerm - (c1 * omegaD + c2 * omegaZeta) * sinTerm) * expTerm
		return newOffset, newVelocity
	end

	if dampingRatio > (1 + DAMPING_EPSILON) then
		local sqrtTerm = math.sqrt(dampingRatio * dampingRatio - 1)
		local r1 = -angularFrequency * (dampingRatio - sqrtTerm)
		local r2 = -angularFrequency * (dampingRatio + sqrtTerm)
		local denominator = r1 - r2
		if math.abs(denominator) <= 1e-6 then
			dampingRatio = 1
		else
			local c1 = (velocity - offset * r2) / denominator
			local c2 = offset - c1
			local exp1 = math.exp(r1 * dt)
			local exp2 = math.exp(r2 * dt)
			local newOffset = c1 * exp1 + c2 * exp2
			local newVelocity = c1 * r1 * exp1 + c2 * r2 * exp2
			return newOffset, newVelocity
		end
	end

	local expTerm = math.exp(-angularFrequency * dt)
	local c1 = offset
	local c2 = velocity + offset * angularFrequency
	local newOffset = (c1 + c2 * dt) * expTerm
	local newVelocity = (velocity - c2 * angularFrequency * dt) * expTerm
	return newOffset, newVelocity
end

function SpringModule.new(target: any, speed: number?, damping: number?, mass: number?, stiffness: number?)
	local self = setmetatable({}, SpringModule)
	self.Target = target
	self.Position = target
	self.Velocity = zeroLike(target)
	self.Speed = speed or 10
	self.Damping = damping or 0.7
	self.Mass = mass or 1
	self.Stiffness = stiffness
	self._useCustomStiffness = stiffness ~= nil
	self.MaxVelocity = typeof(target) == "Vector3" and Vector3.new(1000, 1000, 1000) or 1000
	return self
end

function SpringModule:Update(deltaTime: number): any
	if not deltaTime or deltaTime <= 0 then
		return self.Position
	end

	local dt = math.clamp(deltaTime, 0, 0.1)
	local target = self.Target
	local position = self.Position
	local velocity = self.Velocity
	local damping = self.Damping or 0.7
	local mass = math.max(self.Mass or 1, MIN_MASS)
	local stiffness = self._useCustomStiffness and (self.Stiffness or 0) or (self.Speed * self.Speed)

	local offset = position - target
	if magnitude(offset) < REST_EPSILON and magnitude(velocity) < REST_EPSILON then
		self.Position = target
		self.Velocity = zeroLike(target)
		return self.Position
	end

	if stiffness > 0 then
		local angularFrequency = math.sqrt(stiffness / mass)
		local newOffset, newVelocity = solveSpringStep(offset, velocity, angularFrequency, damping, dt)
		position = target + newOffset
		velocity = newVelocity
	else
		position = position + velocity * dt
	end

	velocity = clampVelocity(velocity, self.MaxVelocity)

	if typeof(position) == "Vector3" then
		if position.Magnitude > 1000 then
			self.Position = target
			self.Velocity = Vector3.new(0, 0, 0)
			return self.Position
		end
	elseif typeof(position) == "number" then
		if math.abs(position) > 1000 then
			self.Position = target
			self.Velocity = 0
			return self.Position
		end
	end

	self.Velocity = velocity
	self.Position = position
	return position
end

function SpringModule:SetTarget(target: any)
	self.Target = target
end

function SpringModule:SetSpeed(speed: number)
	self.Speed = speed
end

function SpringModule:SetDamping(damping: number)
	self.Damping = damping
end

function SpringModule:SetMass(mass: number)
	self.Mass = mass
end

function SpringModule:SetStiffness(stiffness: number)
	self.Stiffness = stiffness
	self._useCustomStiffness = true
end

function SpringModule:SetMaxVelocity(maxVel: any)
	self.MaxVelocity = maxVel
end

function SpringModule:Reset(target: any?)
	self.Target = target or self.Target
	self.Position = self.Target
	self.Velocity = zeroLike(self.Target)
end

function SpringModule:SetValue(value: any)
	self.Position = value
	self.Velocity = zeroLike(value)
end

function SpringModule:GetPosition(): any
	return self.Position
end

function SpringModule:GetVelocity(): any
	return self.Velocity
end

function SpringModule:GetTarget(): any
	return self.Target
end

function SpringModule:AddVelocity(velocityImpulse: any)
	self.Velocity = self.Velocity + velocityImpulse
end

return SpringModule
