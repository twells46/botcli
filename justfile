check:
	g++ -std=c++17 -fsyntax-only \
		-Isrc -Ithird_party \
		-Ilibwallaby/module/accel/public \
		-Ilibwallaby/module/analog/public \
		-Ilibwallaby/module/button/public \
		-Ilibwallaby/module/digital/public \
		-Ilibwallaby/module/gyro/public \
		-Ilibwallaby/module/magneto/public \
		-Ilibwallaby/module/motor/public \
		-Ilibwallaby/module/servo/public \
		src/main.cpp

