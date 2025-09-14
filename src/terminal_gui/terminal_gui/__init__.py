# Azioni
self.arm_action_name = self.params['actions']['arm']
self.disarm_action_name = self.params['actions']['disarm']
self.takeoff_action_name = self.params['actions']['takeoff']
self.landing_action_name = self.params['actions']['landing']
self.navigate_action_name = self.params['actions']['navigate']

self.declare_parameter('arm_action', self.arm_action_name)
self.declare_parameter('disarm_action', self.disarm_action_name)
self.declare_parameter('takeoff_action', self.takeoff_action_name)
self.declare_parameter('landing_action', self.landing_action_name)
self.declare_parameter('navigate_action', self.navigate_action_name)

# Servizi
self.enable_service_name = self.params['services']['enable_component']
self.reset_service_name = self.params['services']['reset_component']
self.declare_parameter('enable_service', self.enable_service_name)
self.declare_parameter('reset_service', self.reset_service_name)

# GUI
self.throttle_default = self.params['gui']['slider_default_values']['throttle']
self.declare_parameter('throttle_default', self.throttle_default)
