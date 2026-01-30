```mermaid
graph TD
    subgraph System Inputs
        HW[System Hardware]
        NAV[Navigation]
        GUID[Guidance]
    end

    subgraph Core Software
        REX[Scheduler / REX]
        SEQ[Sequencer Software]
    end

    subgraph System Outputs
        DAP[Digital Autopilot]
        FUZE[Actuators / Fuze]
        GUID_OUT[Guidance]
    end

    style HW fill:#f9f,stroke:#333,stroke-width:2px
    style NAV fill:#e6f3ff,stroke:#333,stroke-width:2px
    style GUID fill:#e6e6fa,stroke:#333,stroke-width:2px
    style DAP fill:#f9f2e6,stroke:#333,stroke-width:2px
    style FUZE fill:#ffe6e6,stroke:#333,stroke-width:2px
    style GUID_OUT fill:#e6e6fa,stroke:#333,stroke-width:2px
    style SEQ fill:#d4edda,stroke:#333,stroke-width:4px
    style REX fill:#ffe6cc,stroke:#333,stroke-width:2px

    HW -- G-Switch Active --- SEQ

    NAV -- Roll Rate --- REX
    GUID -- tGo Signal --- REX

    REX -- Roll Rate, tGo --- SEQ

    SEQ --"Sequencing Flags<br/>(All Output Commands)"--> REX

    REX -- CONTROL_ON ---> DAP
    REX -- GUID_START ---> GUID_OUT
    REX -- "FSA_FLAG,<br/>CANARD_FLAG,<br/>PROX_ENABLE" ---> FUZE
