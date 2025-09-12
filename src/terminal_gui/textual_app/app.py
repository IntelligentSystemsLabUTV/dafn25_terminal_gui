# app.py
import asyncio
import threading

from textual.app import App, ComposeResult
from textual.widgets import Button, Static, TabbedContent, TabPane

import sys
import os

# Aggiunge il path dove si trova terminal_node.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
from terminal_node import TerminalNode



class TerminalGUI(App):
    """GUI testuale per interagire con il nodo ROS TerminalNode."""

    CSS_PATH = "style.css"

    def __init__(self):
        super().__init__()
        # Inizializza ROS2
        rclpy.init()
        self.node = TerminalNode()

        # Executor multi-thread per ROS2
        from rclpy.executors import MultiThreadedExecutor
        self.executor = MultiThreadedExecutor()

        # Avvia l'executor ROS2 in un thread separato
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def compose(self) -> ComposeResult:
        """Crea la UI con i tab."""
        # Tabs principali
        yield TabbedContent(
            TabPane(
                children=Static("Qui puoi gestire i servizi ROS"),
                title="Services",
                id="tab_services"
            ),
            TabPane(
                children=Static("Qui puoi gestire Arm/Disarm"),
                title="Arm/Disarm",
                id="tab_arm"
            ),
            TabPane(
                children=Static("Qui puoi gestire Takeoff/Landing/Navigate"),
                title="Flight",
                id="tab_flight"
            ),
        )


        # Bottoni al di fuori dei Tab
        yield Button("Arm", id="arm_button")
        yield Button("Disarm", id="disarm_button")
        yield Button("Enable Service", id="enable_service_button")
        yield Button("Reset Service", id="reset_service_button")


        # Bottoni al di fuori dei Tab
        yield Button("Arm", id="arm_button")
        yield Button("Disarm", id="disarm_button")
        yield Button("Enable Service", id="enable_service_button")
        yield Button("Reset Service", id="reset_service_button")

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        """Gestisce la pressione dei bottoni."""
        # Qui puoi aggiungere logica per chiamare le azioni async del nodo
        if event.button.id == "arm_button":
            # Esempio: invia un goal all'azione ARM
            result = await self.node.send_arm_goal(goal_order=3)
            self.console.log(f"Risultato ARM: {result.sequence}")

        elif event.button.id == "disarm_button":
            result = await self.node.send_disarm_goal(goal_order=3)
            self.console.log(f"Risultato DISARM: {result.sequence}")

        elif event.button.id == "enable_service_button":
            result = await self.node.call_enable_service()
            self.console.log(f"Enable Service: {result.success}")

        elif event.button.id == "reset_service_button":
            result = await self.node.call_reset_service()
            self.console.log(f"Reset Service: {result.success}")


if __name__ == "__main__":
    TerminalGUI().run()




#cosa fa
#1. Avvia il nodo ROS2 TerminalNode.
#2. Crea un executor multi-threaded per gestire le azioni e i servizi ROS in parallelo alla GUI.
#3. Definisce tre tab (Services, Arm/Disarm, Flight) con testi statici.
#4. Gestisce eventuali bottoni (che puoi aggiungere in ogni tab) per inviare comandi al nodo ROS2