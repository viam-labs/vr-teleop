package main

import (
	"math"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

const eps = 1e-4

func approxEq(a, b, tol float64) bool { return math.Abs(a-b) < tol }

func quatEq(t *testing.T, label string, got, want mgl64.Quat) {
	t.Helper()
	// q and -q represent the same rotation; normalise sign by W.
	if got.W < 0 {
		got.W = -got.W
		got.V = got.V.Mul(-1)
	}
	if want.W < 0 {
		want.W = -want.W
		want.V = want.V.Mul(-1)
	}
	if !approxEq(got.V[0], want.V[0], eps) || !approxEq(got.V[1], want.V[1], eps) ||
		!approxEq(got.V[2], want.V[2], eps) || !approxEq(got.W, want.W, eps) {
		t.Errorf("%s: got (%.5f, %.5f, %.5f, %.5f), want (%.5f, %.5f, %.5f, %.5f)",
			label, got.V[0], got.V[1], got.V[2], got.W, want.V[0], want.V[1], want.V[2], want.W)
	}
}

func posEq(t *testing.T, label string, got, want [3]float64) {
	t.Helper()
	if !approxEq(got[0], want[0], eps) || !approxEq(got[1], want[1], eps) || !approxEq(got[2], want[2], eps) {
		t.Errorf("%s: got (%.5f, %.5f, %.5f), want (%.5f, %.5f, %.5f)",
			label, got[0], got[1], got[2], want[0], want[1], want[2])
	}
}

// ---------------------------------------------------------------------------
// mat34ToMat4
// ---------------------------------------------------------------------------

// identity rotation matrix → identity Mat4 (rotation part)
func TestMat34ToMat4_Identity(t *testing.T) {
	m := [12]float32{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
	}
	mat := mat34ToMat4(m)
	pos := [3]float64{mat.At(0, 3), mat.At(1, 3), mat.At(2, 3)}
	posEq(t, "pos", pos, [3]float64{0, 0, 0})
	q := mgl64.Mat4ToQuat(mat).Normalize()
	quatEq(t, "rot", q, mgl64.Quat{W: 1})
}

// 90° rotation around Y axis
func TestMat34ToMat4_RotY90(t *testing.T) {
	m := [12]float32{
		0, 0, 1, 0,
		0, 1, 0, 0,
		-1, 0, 0, 0,
	}
	mat := mat34ToMat4(m)
	q := mgl64.Mat4ToQuat(mat).Normalize()
	s := math.Sqrt2 / 2
	quatEq(t, "rot", q, mgl64.Quat{W: s, V: mgl64.Vec3{0, s, 0}})
}

// 90° rotation around X axis
func TestMat34ToMat4_RotX90(t *testing.T) {
	m := [12]float32{
		1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 0,
	}
	mat := mat34ToMat4(m)
	q := mgl64.Mat4ToQuat(mat).Normalize()
	s := math.Sqrt2 / 2
	quatEq(t, "rot", q, mgl64.Quat{W: s, V: mgl64.Vec3{s, 0, 0}})
}

// position is read from column 3
func TestMat34ToMat4_Position(t *testing.T) {
	m := [12]float32{
		1, 0, 0, 1.5,
		0, 1, 0, 2.5,
		0, 0, 1, 3.5,
	}
	mat := mat34ToMat4(m)
	pos := [3]float64{mat.At(0, 3), mat.At(1, 3), mat.At(2, 3)}
	posEq(t, "pos", pos, [3]float64{1.5, 2.5, 3.5})
}

// ---------------------------------------------------------------------------
// applyCalib — position
// ---------------------------------------------------------------------------

func calibState(pos [3]float64, mat mgl64.Mat4) ControllerState {
	return ControllerState{Connected: true, Pos: pos, Mat: mat}
}

func withCalib(yaw float64, fn func()) {
	calibMu.Lock()
	calibYaw = yaw
	calibSet = true
	calibMu.Unlock()
	fn()
	calibMu.Lock()
	calibYaw = 0
	calibSet = false
	calibMu.Unlock()
}

// zero yaw → no change
func TestApplyCalib_ZeroYaw(t *testing.T) {
	cs := calibState([3]float64{1, 2, 3}, mgl64.Ident4())
	withCalib(0, func() {
		out := applyCalib(cs)
		posEq(t, "pos", out.Pos, cs.Pos)
	})
}

// 90° yaw: position (1,0,0) → (0,1,0)
func TestApplyCalib_Pos90(t *testing.T) {
	cs := calibState([3]float64{1, 5, 0}, mgl64.Ident4())
	withCalib(math.Pi/2, func() {
		out := applyCalib(cs)
		posEq(t, "pos", out.Pos, [3]float64{-5, 1, 0})
	})
}

// 180° yaw: position (1,0,0) → (-1,0,0)
func TestApplyCalib_Pos180(t *testing.T) {
	cs := calibState([3]float64{1, 0, 0}, mgl64.Ident4())
	withCalib(math.Pi, func() {
		out := applyCalib(cs)
		posEq(t, "pos", out.Pos, [3]float64{-1, 0, 0})
	})
}

// calib rotation must not affect Z component of position
func TestApplyCalib_ZUnchanged(t *testing.T) {
	cs := calibState([3]float64{3, 7, 2}, mgl64.Ident4())
	withCalib(1.23, func() {
		out := applyCalib(cs)
		if !approxEq(out.Pos[2], 2, eps) {
			t.Errorf("Z position changed: got %.5f, want 2", out.Pos[2])
		}
	})
}

// ---------------------------------------------------------------------------
// computeCalibYaw
// ---------------------------------------------------------------------------

func TestComputeCalibYaw_RoundTrip(t *testing.T) {
	// Build a Y-rotation matrix for 45°.
	angle := math.Pi / 4
	m := mgl64.HomogRotate3DY(angle)

	yaw, ok := computeCalibYaw(m)
	if !ok {
		t.Fatal("computeCalibYaw returned ok=false")
	}

	// Apply the computed yaw to a unit position on the X axis.
	cs := calibState([3]float64{1, 0, 0}, mgl64.Ident4())
	withCalib(yaw, func() {
		out := applyCalib(cs)
		// Just verify it's a unit vector (rotation preserves length).
		mag := math.Sqrt(out.Pos[0]*out.Pos[0] + out.Pos[2]*out.Pos[2])
		if !approxEq(mag, 1, eps) {
			t.Errorf("position magnitude after calib: got %.5f, want 1.0", mag)
		}
	})
}

// ---------------------------------------------------------------------------
// Mat4 vector rotation (replaces qRotateVec tests)
// ---------------------------------------------------------------------------

func TestMat4RotateVec_Identity(t *testing.T) {
	v := mgl64.Ident4().Mul4x1(mgl64.Vec4{1, 2, 3, 0})
	if !approxEq(v[0], 1, eps) || !approxEq(v[1], 2, eps) || !approxEq(v[2], 3, eps) {
		t.Errorf("got (%.5f, %.5f, %.5f), want (1, 2, 3)", v[0], v[1], v[2])
	}
}

func TestMat4RotateVec_90Z(t *testing.T) {
	// 90° around Z: (1,0,0) → (0,1,0)
	m := mgl64.HomogRotate3DZ(math.Pi / 2)
	v := m.Mul4x1(mgl64.Vec4{1, 0, 0, 0})
	if !approxEq(v[0], 0, eps) || !approxEq(v[1], 1, eps) || !approxEq(v[2], 0, eps) {
		t.Errorf("got (%.5f, %.5f, %.5f), want (0, 1, 0)", v[0], v[1], v[2])
	}
}

func TestMat4RotateVec_90X(t *testing.T) {
	// 90° around X: (0,1,0) → (0,0,1)
	m := mgl64.HomogRotate3DX(math.Pi / 2)
	v := m.Mul4x1(mgl64.Vec4{0, 1, 0, 0})
	if !approxEq(v[0], 0, eps) || !approxEq(v[1], 0, eps) || !approxEq(v[2], 1, eps) {
		t.Errorf("got (%.5f, %.5f, %.5f), want (0, 0, 1)", v[0], v[1], v[2])
	}
}

func TestMat4RotateVec_180Y(t *testing.T) {
	// 180° around Y: (1,0,0) → (-1,0,0)
	m := mgl64.HomogRotate3DY(math.Pi)
	v := m.Mul4x1(mgl64.Vec4{1, 0, 0, 0})
	if !approxEq(v[0], -1, eps) || !approxEq(v[1], 0, eps) || !approxEq(v[2], 0, eps) {
		t.Errorf("got (%.5f, %.5f, %.5f), want (-1, 0, 0)", v[0], v[1], v[2])
	}
}

// ---------------------------------------------------------------------------
// Similarity transform T * M * T^-1 (replaces transformToRobotFrame tests)
// ---------------------------------------------------------------------------

func similarityTransform(m, t mgl64.Mat4) mgl64.Mat4 {
	return t.Mul4(m).Mul4(t.Inv())
}

func TestSimilarityTransform_IdentityT(t *testing.T) {
	// T=identity: T*M*T^-1 = M
	T := mgl64.Ident4()
	M := mgl64.HomogRotate3DX(math.Pi / 2)
	got := mgl64.Mat4ToQuat(similarityTransform(M, T)).Normalize()
	want := mgl64.Mat4ToQuat(M).Normalize()
	quatEq(t, "rot", got, want)
}

func TestSimilarityTransform_IdentityM(t *testing.T) {
	// M=identity: T*I*T^-1 = I
	T := mgl64.HomogRotate3DZ(math.Pi / 2)
	M := mgl64.Ident4()
	got := mgl64.Mat4ToQuat(similarityTransform(M, T)).Normalize()
	quatEq(t, "rot", got, mgl64.Quat{W: 1})
}

func TestSimilarityTransform_Conjugation(t *testing.T) {
	// T = 90° around Z, M = 90° around X
	// T*M*T^-1 should be 90° around Y (Z rotates X axis to Y axis)
	T := mgl64.HomogRotate3DZ(math.Pi / 2)
	M := mgl64.HomogRotate3DX(math.Pi / 2)
	got := mgl64.Mat4ToQuat(similarityTransform(M, T)).Normalize()
	want := mgl64.Mat4ToQuat(mgl64.HomogRotate3DY(math.Pi / 2)).Normalize()
	quatEq(t, "rot", got, want)
}

// ---------------------------------------------------------------------------
// lighthouseTransform constant
// ---------------------------------------------------------------------------

func TestLighthouseTransform_Value(t *testing.T) {
	// lighthouseTransform = rotZ(+90°)
	want := mgl64.Mat4ToQuat(mgl64.HomogRotate3DZ(math.Pi / 2)).Normalize()
	got := mgl64.Mat4ToQuat(lighthouseTransform).Normalize()
	quatEq(t, "lighthouseTransform", got, want)
}

func TestLighthouseTransform_BasisVectors(t *testing.T) {
	// libsurvive: X-left, -Y-forward, Z-up
	// Viam:       X-forward, Y-left, Z-up
	// rotZ(+90°): -Y→+X, +X→+Y, +Z→+Z

	// -Y (forward) → +X (robot forward)
	v := lighthouseTransform.Mul4x1(mgl64.Vec4{0, -1, 0, 0})
	if !approxEq(v[0], 1, eps) || !approxEq(v[1], 0, eps) || !approxEq(v[2], 0, eps) {
		t.Errorf("-Y→+X: got (%.5f, %.5f, %.5f), want (1, 0, 0)", v[0], v[1], v[2])
	}

	// +X (left) → +Y (robot left)
	v = lighthouseTransform.Mul4x1(mgl64.Vec4{1, 0, 0, 0})
	if !approxEq(v[0], 0, eps) || !approxEq(v[1], 1, eps) || !approxEq(v[2], 0, eps) {
		t.Errorf("+X→+Y: got (%.5f, %.5f, %.5f), want (0, 1, 0)", v[0], v[1], v[2])
	}

	// +Z (up) → +Z (robot up)
	v = lighthouseTransform.Mul4x1(mgl64.Vec4{0, 0, 1, 0})
	if !approxEq(v[0], 0, eps) || !approxEq(v[1], 0, eps) || !approxEq(v[2], 1, eps) {
		t.Errorf("+Z→+Z: got (%.5f, %.5f, %.5f), want (0, 0, 1)", v[0], v[1], v[2])
	}
}

// ---------------------------------------------------------------------------
// quatToOVDeg
// ---------------------------------------------------------------------------

func TestQuatToOVDeg_Identity(t *testing.T) {
	ox, oy, oz, th := quatToOVDeg(mgl64.Quat{W: 1})
	if !approxEq(ox, 0, eps) || !approxEq(oy, 0, eps) || !approxEq(oz, 1, eps) {
		t.Errorf("axis: got (%.5f, %.5f, %.5f), want (0, 0, 1)", ox, oy, oz)
	}
	if !approxEq(th, 0, eps) {
		t.Errorf("theta: got %.5f, want 0", th)
	}
}

func TestQuatToOVDeg_180Z(t *testing.T) {
	ox, oy, oz, th := quatToOVDeg(mgl64.Quat{W: 0, V: mgl64.Vec3{0, 0, 1}})
	if !approxEq(ox, 0, eps) || !approxEq(oy, 0, eps) || !approxEq(oz, 1, eps) {
		t.Errorf("axis: got (%.5f, %.5f, %.5f), want (0, 0, 1)", ox, oy, oz)
	}
	if !approxEq(math.Abs(th), 180, eps) {
		t.Errorf("theta: got %.5f, want ±180", th)
	}
}

func TestQuatToOVDeg_90Y(t *testing.T) {
	s := math.Sqrt2 / 2
	ox, oy, oz, th := quatToOVDeg(mgl64.Quat{W: s, V: mgl64.Vec3{0, s, 0}})
	if !approxEq(ox, 1, eps) || !approxEq(oy, 0, eps) || !approxEq(oz, 0, eps) {
		t.Errorf("axis: got (%.5f, %.5f, %.5f), want (1, 0, 0)", ox, oy, oz)
	}
	if !approxEq(th, 0, eps) {
		t.Errorf("theta: got %.5f, want 0", th)
	}
}

func TestQuatToOVDeg_90X(t *testing.T) {
	s := math.Sqrt2 / 2
	ox, oy, oz, th := quatToOVDeg(mgl64.Quat{W: s, V: mgl64.Vec3{s, 0, 0}})
	if !approxEq(ox, 0, eps) || !approxEq(oy, -1, eps) || !approxEq(oz, 0, eps) {
		t.Errorf("axis: got (%.5f, %.5f, %.5f), want (0, -1, 0)", ox, oy, oz)
	}
	if !approxEq(th, 90, eps) {
		t.Errorf("theta: got %.5f, want 90", th)
	}
}

func TestQuatToOVDeg_Neg90X(t *testing.T) {
	s := math.Sqrt2 / 2
	ox, oy, oz, th := quatToOVDeg(mgl64.Quat{W: s, V: mgl64.Vec3{-s, 0, 0}})
	if !approxEq(ox, 0, eps) || !approxEq(oy, 1, eps) || !approxEq(oz, 0, eps) {
		t.Errorf("axis: got (%.5f, %.5f, %.5f), want (0, 1, 0)", ox, oy, oz)
	}
	if !approxEq(th, -90, eps) {
		t.Errorf("theta: got %.5f, want -90", th)
	}
}

func TestQuatToOVDeg_90Z(t *testing.T) {
	s := math.Sqrt2 / 2
	ox, oy, oz, th := quatToOVDeg(mgl64.Quat{W: s, V: mgl64.Vec3{0, 0, s}})
	if !approxEq(ox, 0, eps) || !approxEq(oy, 0, eps) || !approxEq(oz, 1, eps) {
		t.Errorf("axis: got (%.5f, %.5f, %.5f), want (0, 0, 1)", ox, oy, oz)
	}
	if !approxEq(th, 90, eps) {
		t.Errorf("theta: got %.5f, want 90", th)
	}
}

// ---------------------------------------------------------------------------
// quatToOVDeg — Viam reference test vectors
// ---------------------------------------------------------------------------

func ovEq(t *testing.T, label string, gotOX, gotOY, gotOZ, gotTh, wantOX, wantOY, wantOZ, wantThDeg float64) {
	t.Helper()
	const tol = 1e-3
	if !approxEq(gotOX, wantOX, tol) || !approxEq(gotOY, wantOY, tol) || !approxEq(gotOZ, wantOZ, tol) {
		t.Errorf("%s axis: got (%.6f, %.6f, %.6f), want (%.6f, %.6f, %.6f)",
			label, gotOX, gotOY, gotOZ, wantOX, wantOY, wantOZ)
	}
	if !approxEq(math.Abs(gotTh), math.Abs(wantThDeg), tol) {
		t.Errorf("%s theta: got %.6f, want %.6f", label, gotTh, wantThDeg)
	}
}

// q helper: builds mgl64.Quat from (x, y, z, w) for brevity.
func q(x, y, z, w float64) mgl64.Quat {
	return mgl64.Quat{W: w, V: mgl64.Vec3{x, y, z}}
}

func TestQuatToOVDeg_ViamRef(t *testing.T) {
	rad2deg := 180.0 / math.Pi

	// Case 1: quat(0.707, 0, 0, 0.707) → OV(0, -1, 0, π/2)
	ox, oy, oz, th := quatToOVDeg(q(0.7071067811865476, 0, 0, 0.7071067811865476))
	ovEq(t, "case1", ox, oy, oz, th, 0, -1, 0, 1.5707963267948966*rad2deg)

	// Case 2: quat(-0.707, 0, 0, 0.707) → OV(0, 1, 0, -π/2)
	ox, oy, oz, th = quatToOVDeg(q(-0.7071067811865476, 0, 0, 0.7071067811865476))
	ovEq(t, "case2", ox, oy, oz, th, 0, 1, 0, -1.5707963267948966*rad2deg)

	// Case 3: quat(0, -0.28, 0, 0.96) → OV(-0.5376, 0, 0.8432, -π)
	ox, oy, oz, th = quatToOVDeg(q(0, -0.28, 0, 0.96))
	ovEq(t, "case3", ox, oy, oz, th, -0.5376, 0, 0.8432, -math.Pi*rad2deg)

	// Case 4: quat(0, 0, -0.28, 0.96) → OV(0, 0, 1, -0.5676 rad)
	ox, oy, oz, th = quatToOVDeg(q(0, 0, -0.28, 0.96))
	ovEq(t, "case4", ox, oy, oz, th, 0, 0, 1, -0.5675882184166557*rad2deg)

	// Case 5: quat(-0.28, 0, 0, 0.96) → OV(0, 0.5376, 0.8432, -π/2)
	ox, oy, oz, th = quatToOVDeg(q(-0.28, 0, 0, 0.96))
	ovEq(t, "case5", ox, oy, oz, th, 0, 0.5376, 0.8432, -1.5707963267948966*rad2deg)

	// Case 6: quat(0.28, 0, 0, 0.96) → OV(0, -0.5376, 0.8432, π/2)
	ox, oy, oz, th = quatToOVDeg(q(0.28, 0, 0, 0.96))
	ovEq(t, "case6", ox, oy, oz, th, 0, -0.5376, 0.8432, 1.5707963267948966*rad2deg)

	// Case 7: quat(-0.5, -0.5, -0.5, 0.5) → OV(0, 1, 0, -π)
	ox, oy, oz, th = quatToOVDeg(q(-0.5, -0.5, -0.5, 0.5))
	ovEq(t, "case7", ox, oy, oz, th, 0, 1, 0, -math.Pi*rad2deg)

	// Case 8: arbitrary quat → OV(0.5048, 0.5890, 0.6311, 0.02 rad)
	ox, oy, oz, th = quatToOVDeg(q(-0.17555966025413142, 0.39198397193979817, 0.3855375485164001, 0.816632212270443))
	ovEq(t, "case8", ox, oy, oz, th, 0.5048437942940054, 0.5889844266763397, 0.631054742867507, 0.02*rad2deg)
}

// ---------------------------------------------------------------------------
// Dead-zone filtering
// ---------------------------------------------------------------------------

func TestExceedsDeadzone_NilLastSent(t *testing.T) {
	p := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	if !exceedsDeadzone(nil, p, 0.5, 1.0) {
		t.Error("expected true for nil lastSent (first frame)")
	}
}

func TestExceedsDeadzone_IdenticalPose(t *testing.T) {
	p := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 45}
	if exceedsDeadzone(&p, p, 0.5, 1.0) {
		t.Error("expected false for identical poses")
	}
}

func TestExceedsDeadzone_PositionAbove(t *testing.T) {
	last := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	next := pose{x: 101, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	if !exceedsDeadzone(&last, next, 0.5, 1.0) {
		t.Error("expected true: position delta 1mm > threshold 0.5mm")
	}
}

func TestExceedsDeadzone_PositionBelow(t *testing.T) {
	last := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	next := pose{x: 100.3, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	if exceedsDeadzone(&last, next, 0.5, 1.0) {
		t.Error("expected false: position delta 0.3mm < threshold 0.5mm")
	}
}

func TestExceedsDeadzone_RotationAbove(t *testing.T) {
	last := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	next := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 5}
	if !exceedsDeadzone(&last, next, 0.5, 1.0) {
		t.Error("expected true: rotation delta 5deg > threshold 1deg")
	}
}

func TestExceedsDeadzone_RotationBelow(t *testing.T) {
	last := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	next := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0.3}
	if exceedsDeadzone(&last, next, 0.5, 1.0) {
		t.Error("expected false: rotation delta 0.3deg < threshold 1deg")
	}
}

func TestExceedsDeadzone_BothZeroDisablesFilter(t *testing.T) {
	last := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	next := pose{x: 100.001, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0.001}
	if !exceedsDeadzone(&last, next, 0, 0) {
		t.Error("expected true: both thresholds 0 disables filtering")
	}
}

func TestExceedsDeadzone_OnlyPosThreshold(t *testing.T) {
	last := pose{x: 100, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	next := pose{x: 100.1, y: 200, z: 300, ox: 0, oy: 0, oz: 1, thetaDeg: 0}
	// rotDeadzone=0 means rotation is always "exceeds", so any tiny move sends
	if !exceedsDeadzone(&last, next, 0.5, 0) {
		t.Error("expected true: rot threshold 0 disables rot filtering")
	}
}

// --------------- EMA smoothing tests ---------------

func TestEmaSmooth_NilPrev(t *testing.T) {
	raw := pose{x: 10, y: 20, z: 30, ox: 0.1, oy: 0.2, oz: 0.97, thetaDeg: 45}
	got := emaSmooth(nil, raw, 0.5)
	if got != raw {
		t.Errorf("nil prev: got %+v, want %+v", got, raw)
	}
}

func TestEmaSmooth_Alpha1(t *testing.T) {
	prev := pose{x: 100, y: 200, z: 300, ox: 1, oy: 0, oz: 0, thetaDeg: 90}
	raw := pose{x: 10, y: 20, z: 30, ox: 0, oy: 0, oz: 1, thetaDeg: 45}
	got := emaSmooth(&prev, raw, 1.0)
	if got != raw {
		t.Errorf("alpha=1: got %+v, want %+v", got, raw)
	}
}

func TestEmaSmooth_HalfAlpha(t *testing.T) {
	prev := pose{x: 0, y: 0, z: 0, ox: 0, oy: 0, oz: 0, thetaDeg: 0}
	raw := pose{x: 10, y: 20, z: 30, ox: 0.4, oy: 0.6, oz: 0.8, thetaDeg: 90}
	got := emaSmooth(&prev, raw, 0.5)
	const eps = 1e-9
	check := func(name string, got, want float64) {
		if math.Abs(got-want) > eps {
			t.Errorf("%s: got %f, want %f", name, got, want)
		}
	}
	check("x", got.x, 5)
	check("y", got.y, 10)
	check("z", got.z, 15)
	check("ox", got.ox, 0.2)
	check("oy", got.oy, 0.3)
	check("oz", got.oz, 0.4)
	check("thetaDeg", got.thetaDeg, 45)
}

func TestEmaSmooth_Convergence(t *testing.T) {
	target := pose{x: 100, y: 100, z: 100, ox: 0, oy: 0, oz: 1, thetaDeg: 90}
	var smoothed *pose
	for i := 0; i < 50; i++ {
		result := emaSmooth(smoothed, target, 0.3)
		smoothed = &result
	}
	const eps = 0.01
	if math.Abs(smoothed.x-100) > eps || math.Abs(smoothed.thetaDeg-90) > eps {
		t.Errorf("did not converge: got x=%.4f theta=%.4f", smoothed.x, smoothed.thetaDeg)
	}
}
