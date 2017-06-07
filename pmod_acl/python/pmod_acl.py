import time
from pynq import MMIO
from pynq.iop import request_iop
from pynq.iop import iop_const
from pynq.iop import PMODA
from pynq.iop import PMODB

PMOD_ACL_PROGRAM = "pmod_acl.bin"
PMOD_ACL_LOG_START = iop_const.MAILBOX_OFFSET+16
PMOD_ACL_LOG_END = PMOD_ACL_LOG_START+(1000*4)

class Pmod_ACL(object):
	def __init__(self, if_id):
		if not if_id in [PMODA, PMODB]:
			raise ValueError("No such IOP for Pmod device.")
			
		self.iop = request_iop(if_id, PMOD_ACL_PROGRAM)
		self.mmio = self.iop.mmio
		self.log_interval_ms = 1000
		self.iop.start()
	def read(self):
		"""Read current light value measured by the ACL Pmod.

		Returns
		-------
		int
			The current sensor value.

		"""
		self.mmio.write(iop_const.MAILBOX_OFFSET +
						iop_const.MAILBOX_PY2IOP_CMD_OFFSET, 0x3)      
		while (self.mmio.read(iop_const.MAILBOX_OFFSET +
								iop_const.MAILBOX_PY2IOP_CMD_OFFSET) == 0x3):
			pass
		mx = self._reg2int(self.mmio.read(iop_const.MAILBOX_OFFSET))
		my = self._reg2int(self.mmio.read(iop_const.MAILBOX_OFFSET+4))
		mz = self._reg2int(self.mmio.read(iop_const.MAILBOX_OFFSET+8))
		return [float("{0:.2f}".format(mx*0.004)),
				float("{0:.2f}".format(my*0.004)),
				float("{0:.2f}".format(mz*0.004))]

	def _reg2int(self, reg):
		"""Converts 32-bit register value to signed integer in Python.

		Parameters
		----------
		reg: int
			A 32-bit register value read from the mailbox.
			
		Returns
		-------
		int
			A signed integer translated from the register value.

		"""
		result = -(reg>>31 & 0x1)*(1<<31)
		for i in range(31):
			result += (reg>>i & 0x1)*(1<<i)
		return result
