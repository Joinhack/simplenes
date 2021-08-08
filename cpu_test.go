package simplenec

import (
	"testing"
)

func TestZero(t *testing.T) {
	cpu := &CPU{}
	cpu.setZ(1)
	if cpu.z != 0 {
		t.Fail()
	}
}
