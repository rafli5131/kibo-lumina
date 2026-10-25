# Kibo Lumina — Astrobee Robotics Mission

A Java/Android robotics project built around the JAXA Kibo Robot Programming Challenge API. Its mission implementation controls an Astrobee robot through target-navigation logic, camera-based vision processing, and QR-code scanning.

## What the project covers

- Mission planning and movement through the Kibo RPC service.
- Target prioritization under a mission time limit.
- Camera utilities using OpenCV, including ArUco-related operations.
- QR-code recognition and mission completion reporting.

The primary implementation lives in [`YourService.java`](app/src/main/java/jp/jaxa/iss/kibo/rpc/sampleapk/YourService.java). The source implements `runPlan1()`; `runPlan2()` and `runPlan3()` remain placeholders.

## Technology

Java, Android/Gradle, OpenCV, the Kibo RPC API, and Astrobee-related libraries included in this repository.

## Explore and build

1. Clone this repository.
2. Open the project in Android Studio with a compatible JDK and Android SDK.
3. Review `app/build.gradle` and the included `kibo_rpc_api/` and `guest_science_library/` modules.
4. Run the Gradle build using the project's wrapper, then test the service using the Kibo-supported simulation or deployment environment.

```bash
git clone https://github.com/rafli5131/kibo-lumina.git
cd kibo-lumina
./gradlew assembleDebug
```

The build command is provided as a starting point; compatibility depends on the installed Android toolchain and challenge libraries.

## Repository layout

- `app/` — Android application and mission service.
- `kibo_rpc_api/` — Kibo RPC API dependency.
- `guest_science_library/` — supporting Astrobee guest-science library.
- `gradle/` — Gradle wrapper files.

*Educational robotics work; no on-orbit flight validation is claimed.*
