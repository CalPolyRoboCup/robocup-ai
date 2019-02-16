class Strategy:
	zoneDict = {
				1: self.rule_zone_1,
				2: self.rule_zone_2,
				3: self.rule_zone_3,
				4: self.rule_zone_4,
				5: self.rule_zone_5,
				6: self.rule_zone_6,
				7: self.rule_zone_7,
				8: self.rule_zone_9,
				9: self.rule_zone_9,
				10: self.rule_zone_10,
				11: self.rule_zone_11,
				12: self.rule_zone_12,
				13: self.rule_zone_13,
				14: self.rule_zone_14,
				15: self.rule_zone_15,
				16: self.rule_zone_16,
				17: self.rule_zone_17,
				18: self.rule_zone_18,
				19: self.rule_zone_19,
				20: self.rule_zone_20
			}

	def rule_zone_1(self, Player p):
		pass

	def rule_zone_2(self, Player p):
		pass

	def rule_zone_3(self, Player p):
		pass

	def rule_zone_4(self, Player p):
		pass

	def rule_zone_5(self, Player p):
		pass

	def rule_zone_6(self, Player p):
		pass

	def rule_zone_7(self, Player p):
		pass

	def rule_zone_8(self, Player p):
		pass

	def rule_zone_9(self, Player p):
		pass

	def rule_zone_10(self, Player p):
		pass

	def rule_zone_11(self, Player p):
		pass

 	def rule_zone_12(self, Player p):
		pass

	def rule_zone_13(self, Player p):
		pass

	def rule_zone_14(self, Player p):
		pass

	def rule_zone_15(self, Player p):
		pass

	def rule_zone_16(self, Player p):
		pass

	def rule_zone_17(self, Player p):
		pass

	def rule_zone_18(self, Player p):
		pass

	def rule_zone_19(self, Player p):
		pass

	def rule_zone_20(self, Player p):
		pass

	def apply(self, Player p):
		Strategy.zoneDict[p.zone](p)