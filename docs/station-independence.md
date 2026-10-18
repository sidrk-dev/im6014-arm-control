# Dashboard and owner lifecycle

`ArmManualSession` owns the serial worker. `ArmDashboardServer` receives that owner and handles HTTP ingress; restarting this HTTP layer with the same supplied owner does not construct another worker or connection.

`run_dashboard` owns cleanup of its supplied or internally created session. A server bind failure closes that owner. A failure in the HTTP loop closes the server and owner. A failure during server cleanup still attempts owner cleanup. Existing owner close behavior determines whether Stop and resource closure were confirmed; this wrapper does not fabricate confirmation or automatically rearm.

Input expiry runs in the owner. A disconnected browser cannot renew its input lease. Restarting the HTTP layer and reading state cannot rearm a stopped owner, renew an old ticket or transfer authority to a new observer session. Opening an observer session does not itself interrupt an existing valid owner. Movement requires the existing explicit Enable flow.

`test_station_independence.py` uses the actual owner with DemoSerial, rejecting physical constructors and discovery. It checks server construction and cleanup failures, three-joint Stop attempts, keyboard interruption, owner-thread write identity, input expiry without HTTP, HTTP restart and observer authority. Existing C01 checks cover cooperating port exclusion separately.

These results establish software behavior. They do not establish physical stopping time or communication with replacement hardware. Pure JointCore integration with this production owner remains a separate task.
