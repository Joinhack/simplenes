package simplenec

type memory interface {
	read(address uint16) byte
	write(address uint16, value byte)
}
