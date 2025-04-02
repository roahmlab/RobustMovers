# Controllers

Different controller implementations and their inheritance relationships:
```mermaid
graph TD
    A[controller]
    A --> B[controller_model]
    A --> C[controller_interval]
    A --> I[controller_PID]
    B --> D[controller_passivity]
    B --> E[controller_adaptive]
    C --> F[controller_althoff]
    C --> G[controller_armour]
    C --> H[controller_armour]
```

The structure is the same for operational space controllers.