# Receiver-Side Anti-Jamming for Civilian GNSS, 2015–2026: A Structured Narrative Review with Evidence Map and Failure-Mode Synthesis

**Author:** [Author Placeholder]
**Affiliation:** [Affiliation Placeholder]
**Date:** 2026-05-12
**Estimated body word count:** ~7,200 words

---

## Abstract

**Background:** Civilian positioning, navigation, and timing (PNT) infrastructure faces persistent jamming threats from chirp-modulated personal privacy devices (PPDs), military-adjacent high-power sources, and pulsed aeronautical radionavigation signals. Operationally confirmed GNSS outages over the Eastern Mediterranean and Baltic regions, documented by independent field and satellite monitoring campaigns, have elevated anti-jamming receiver design from a niche concern to a safety-critical engineering priority.

**Objective:** This review synthesises civilian global navigation satellite system (GNSS) anti-jamming research from January 2015 to May 2026, addressing three questions: (1) What constitutes the current jammer threat taxonomy? (2) How do mitigation techniques distribute across the receiver processing chain? (3) What are the dominant failure modes and deployment limitations?

**Methods:** A structured narrative review aligned with PRISMA-ScR reporting standards was conducted across IEEE Xplore, Scopus, Web of Science, and Google Scholar. Fifty-eight verified sources spanning peer-reviewed journals, conference proceedings, and gray literature were organised into six thematic domains and evaluated against a source-grading rubric (Levels I–VII). An evidence map (Technique Stage × Jammer Type) and a failure-mode provenance taxonomy were constructed as the primary analytical outputs.

**Key findings:** Chirp PPDs remain the empirically dominant commercial threat, but high-power wideband sources produce the most operationally consequential outages. Mitigation evidence is strongest for spatial processing (controlled reception pattern antenna; CRPA) and pre-correlation excision at L1 C/A frequencies, and weakest for machine learning (ML) and deep learning (DL) methods in real-world conditions. Adaptive notch filters (ANF) introduced a 1–5 m pseudorange bias under strong jamming in one hardware-in-the-loop (HIL) study using a USRP2 plus custom MATLAB experimental chain (single-group, JRC-authored result), arguing against ANF as the sole mitigation for precision applications within that experimental envelope; extrapolation to other receiver platforms requires explicit re-characterisation. Vector tracking and inertial navigation system (INS) deep coupling provide single-digit decibel resilience gains whose precise magnitudes carry legacy-reference caveats. No study in the corpus demonstrates a full combined-mitigation architecture against high-power wideband sources at field level.

**Conclusion:** Receiver-side anti-jamming has progressed from single-technique demonstrations to architecturally integrated multi-layer systems, but a persistent gap between controlled-environment performance and field deployment readiness limits operational translation. Priority gaps include updated PPD characterisation, real-jammer ML training datasets, and full-stack HIL evaluation against wideband threats.

**Keywords:** GNSS, GPS, anti-jamming, CRPA, interference mitigation, civilian receivers, narrative review, evidence map

---

## 1. Introduction

Modern civilian society depends on GNSS-derived PNT to a degree that has no historical precedent. Aviation precision approach, maritime harbour navigation, railway positive train control, cellular network synchronisation, financial transaction timestamping, and emergency dispatch routing all rely on GPS or multi-constellation GNSS signals whose received power at the antenna is approximately −130 dBm, roughly 20 dB below thermal noise in a 2 MHz bandwidth. This inherent fragility is not a design flaw that future signal upgrades will eliminate; it is a consequence of the inverse-square law and the finite power budget of medium-Earth orbit satellites. Any transmitter within line-of-sight that radiates more than a few milliwatts in the L-band can degrade or deny GNSS reception across a substantial area.

The operational reality of that vulnerability has been established by converging independent measurement campaigns. Pullen and Gao (2012) documented how a single USD 33 PPD in a commuter vehicle repeatedly disabled ground-based augmentation system (GBAS) reference receivers at Newark Liberty Airport, a case that now anchors every civilian anti-jamming survey. Murrian et al. (2021) used a software receiver aboard the International Space Station to construct a three-year global map of jamming zones, geolocating a Syrian source to sub-kilometre accuracy and identifying recurrent interference in the Eastern Mediterranean, Baltic, and Black Sea. Osechas et al. (2022) flew an instrumented Airbus A320 through the same Eastern Mediterranean corridors and recorded full GNSS outages in commercial multimode receivers, with no carrier-to-noise density ratio (C/N0) information available to the flight crew. Felux, Fol, Figuet, Waltert, and Olive (2024) extended the picture using crowd-sourced Automatic Dependent Surveillance-Broadcast (ADS-B) data to map jamming impact across European aviation for the full calendar year 2022, identifying three structurally distinct interference zones with different temporal activity patterns.

These four studies share a common message: jamming of civilian GNSS is not a theoretical threat scenario. It is an ongoing operational condition in specific geographic regions, and its consequences extend from nuisance-level navigation warnings to full avionics failure modes.

Against this backdrop, receiver-side mitigation has been an active research area for over two decades. The existing literature is substantial but fragmented: individual studies characterise specific techniques at specific receiver-chain stages, and technique-level surveys (Morales-Ferre et al., 2020; Ioannides et al., 2016; Gao et al., 2016) catalogue available methods without synthesising the receiver chain as a coupled architecture or explicitly distinguishing what has been field-validated from what remains simulation-predicted. The present review addresses both gaps.

Three research questions organise the synthesis:

- **Sub-RQ1 (Threat taxonomy):** What jammer types, power levels, and operational patterns constitute the current threat to civilian GNSS?
- **Sub-RQ2 (Mitigation techniques):** How are interference mitigation techniques distributed across the receiver chain, and what is the evidence quality for each stage?
- **Sub-RQ3 (Failure modes):** What are the dominant failure modes and deployment limitations of current mitigation techniques, and can they be distinguished by evidence provenance?

The analytical novelty of this review lies in two deliverables constructed from the synthesised corpus: an evidence map tabulating coverage across Technique Stage × Jammer Type × Metric dimensions, and a failure-mode provenance taxonomy that distinguishes observed-incident, HIL-demonstrated, and theory-predicted failure modes. Together, these allow practitioners to read off both the state of mitigation and the confidence basis for each cell, a format that prior surveys have not provided.

The paper proceeds as follows: Section 2 describes the review methodology. Section 3 provides a brief receiver chain primer and defines key terminology. Sections 4–9 present findings by theme (T1–T6), followed by the evidence map (Section 10) and failure-mode synthesis (Section 11). Section 12 discusses cross-theme integration, contradiction resolutions, a platform-differentiated architecture recommendation, and review limitations. Section 13 concludes with knowledge gaps and practitioner guidance.

---

## 2. Methodology

### 2.1 Review Design

This review follows a structured narrative design aligned with PRISMA Extension for Scoping Reviews (PRISMA-ScR) reporting guidance to ensure search transparency. A structured narrative design was selected over either a strict scoping review or a full PRISMA-compliant systematic review with quantitative meta-analysis for two reasons specific to this corpus. First, the corpus exhibits pervasive metric incommensurability: studies report C/N0 recovery in decibels, jamming-to-noise ratio (J/N) tolerance in decibels, acquisition probability as a percentage, tracking error in radians, position error in metres, clock bias in nanoseconds, or signal-to-interference-plus-noise ratio (SINR) improvement in decibels, under incomparable jammer types, front-end configurations, and evaluation environments. No cross-metric ranking, pooled effect size, or quantitative synthesis is feasible or attempted, eliminating meta-analysis as an option. Second, a strict scoping review would catalogue what exists without producing the integrative architectural recommendations and failure-mode taxonomy that the research questions require; the structured narrative form retains PRISMA-ScR search transparency while permitting cross-source thematic integration. The trade-off accepted is that the synthesis carries irreducible interpretive judgement, which is exposed via the evidence map (Table 1), the explicit single-group / multi-group cell coding, and the failure-mode provenance tagging.

### 2.2 Search Strategy

Four databases were searched: IEEE Xplore (including *Transactions on Aerospace and Electronic Systems*, *Proceedings of the IEEE*, *IEEE/ION PLANS* proceedings), Scopus, Web of Science, and Google Scholar. The Institute of Navigation (ION) publication database was queried separately for *NAVIGATION: Journal of the ION*, ION GNSS+ proceedings, and the ION ITM and PLANS series. *GPS Solutions* (Springer) was searched via SpringerLink. Gray literature was retrieved from the JRC Publications Repository for JRC Technical Reports, and Inside GNSS for practitioner features.

**Boolean search strings (representative):**
- ("GPS" OR "GNSS") AND ("jamming" OR "interference") AND ("mitigation" OR "suppression" OR "anti-jamming") AND "receiver"
- ("CRPA" OR "controlled reception pattern antenna" OR "null steering" OR "STAP") AND GNSS
- ("adaptive notch filter" OR "ANF" OR "RIM" OR "robust interference mitigation") AND GNSS
- ("vector tracking" OR "VDLL" OR "VFLL") AND jamming
- ("deep learning" OR "CNN" OR "LSTM") AND GNSS AND interference
- ("L5" OR "E5a" OR "L2C") AND ("jamming" OR "DME" OR "TACAN" OR "blanking")

**Date range:** January 2015 to May 2026 for primary sources; pre-2015 foundational anchors were permitted without date restriction.

### 2.3 Inclusion and Exclusion Criteria

*Included:* Peer-reviewed journal articles, conference papers, and peer-equivalent gray literature; English language; civilian receiver scope (GPS L1 C/A, L2C, L5, Galileo E1/E5a, BeiDou B1/B3); receiver-side mitigation focus; at least one measurable performance metric. Spoofing-only papers were excluded; papers treating jamming and spoofing jointly were included where the jamming aspects provided measurable anti-jamming evidence, with spoofing content noted but not synthesised.

*Excluded:* Military P(Y)/M-code techniques not transferable to civilian receivers; non-English publications; transmitter-side design papers; purely secondary reviews where all primary claims derived from already-included sources.

A multi-database search identified candidate records, which were reduced through duplicate removal, title/abstract screening, and full-text eligibility assessment to the final corpus. The flow (PRISMA-ScR style): 400 records identified via database search (IEEE Xplore ≈ 180, Scopus ≈ 110, Web of Science ≈ 65, Google Scholar ≈ 30, ION ≈ 15) → 73 duplicates removed → 327 unique records screened on title/abstract → 244 excluded at screening (off-topic, spoofing-only, military-only, non-English, or non-peer-equivalent) → 83 records assessed full-text → 25 excluded at full-text (no measurable performance metric, secondary review, or transmitter-side design) → 58 records included in synthesis. Per-database counts are approximate due to overlap between IEEE Xplore and Scopus indexing of the same proceedings.

### 2.4 Source-Grading Rubric

Sources were assigned evidence levels I–VII: Level I (randomised or systematically replicated multi-site field trial), Level II (real-world field measurement or instrumented flight test), Level III (HIL with real jammer signals and real receiver hardware), Level IV (controlled lab with hardware simulator, multiple conditions), Level V (thorough simulation with analytical validation), Level VI (expert consensus guideline), Level VII (textbook, survey, or review paper).

Tier downgrades recommended by the source verification agent were applied: Mitch et al. (2011) from Level II to Level III because the study characterises jammer emissions rather than testing receiver responses; Morong et al. (2019) from Level II to Level III because the study characterises propagation range rather than mitigation effectiveness.

### 2.5 Analytical Methods

**Evidence map:** Technique Stage (RF/antenna, ADC/pre-correlation, baseband, tracking, navigation filter, ML) was crossed with Jammer Type (CW, narrowband, wideband, chirp/swept, pulsed DME/TACAN) and primary performance metric. Each cell was coded: **Strong** (field trial or HIL, multiple independent groups), **Moderate** (HIL or controlled lab, at least two sources from at least two groups), **Single-group** (HIL/lab available from one group, no independent replication), **Sim** (simulation only), or **Gap** (no source in corpus).

**Failure-mode provenance taxonomy:** Each identified failure mode was classified as *[Observed-incident]* (documented in real deployment with field or incident data), *[HIL-demonstrated]* (reproduced in hardware-in-the-loop with real or realistic signals), or *[Theory-predicted]* (follows from analysis or simulation but not experimentally confirmed in HIL or field within this corpus).

### 2.6 Limitations of the Method

The English-only restriction excludes active Chinese-language BeiDou anti-jamming literature and German-language DLR contributions that have not been published in English. The JRC research group (Borio, Gioia, Garcia-Peña) accounts for approximately 16% of the corpus (entries #9, #24, #25, #27, #30, #31, #32, #42, #43), creating a residual corroboration risk for JRC-sourced quantitative claims. The May 2026 cutoff means the synthesis does not capture the most recent ML/DL publications. Eighty-six percent of the corpus was not individually spot-checked beyond a structural audit, carrying residual author-substitution risk: the Felux et al. (2024) author-field correction (the original bibliography had attributed the paper to entirely different authors) serves as a concrete illustration of this risk.

---

## 3. Background: Receiver Chain Primer

A civilian GNSS receiver processes signals through five broad stages: RF/antenna front-end → analogue-to-digital conversion (ADC) and pre-correlation processing → baseband correlator → tracking loops → navigation filter. Jamming interacts differently at each stage, and the choice of mitigation technique is architecturally tied to its insertion point.

**RF/antenna stage.** The signal arrives from a controlled reception pattern antenna (CRPA) or omnidirectional patch antenna. AGC circuitry maintains ADC input level within dynamic range; under strong jamming, AGC compresses both the jammer and the desired signal, reducing the effective signal-to-noise ratio before any digital mitigation can act. The CRPA replaces the omnidirectional antenna with an array whose complex weights can be steered to place pattern nulls on interferers while maintaining gain toward GNSS satellites. Key metrics at this stage: SINR improvement (dB), null depth (dB), and carrier-phase bias (m) introduced by weight application.

**ADC/pre-correlation stage.** Digitised samples are processed before the code correlator sees them. Pulse blanking sets samples exceeding a threshold to zero; frequency-domain adaptive filtering (FDAF) excises spectral bins containing interference energy; both operate without knowledge of the desired GNSS signal structure. Key metrics: C/N0 recovery (dB), acquisition probability (%), blanking duty cycle (%).

**Baseband correlator stage.** After despreading, interference residuals are addressed by adaptive notch filters (ANF) or robust interference mitigation (RIM) using zero-memory nonlinearities (ZMNL). Key metrics: pseudorange bias (m), C/N0 recovery (dB), jamming-to-signal ratio (JSR) tolerance.

**Tracking loops.** Phase-locked loops (PLLs) and delay-locked loops (DLLs) maintain code and carrier phase lock under interference. Adaptive loop bandwidth control (LBCA) and direct-state Kalman filter (DSKF) tracking reduce sensitivity to noise; vector delay/frequency lock loops (VDLL/VFLL) couple multiple satellite channels through a common navigation state. Key metrics: phase tracking error (rad), J/N tolerance at lock threshold (dB), loop bandwidth (Hz).

**Navigation filter.** The Kalman filter fuses pseudoranges and, in tightly or deeply coupled architectures, inertial measurement unit (IMU) outputs from microelectromechanical systems (MEMS). Fault detection and exclusion (FDE) and receiver autonomous integrity monitoring (RAIM) remove degraded measurements. Key metrics: position error (m), bridging duration (s), mean time between integrity events (MTIE).

Jammer types referenced throughout: continuous wave (CW): a single sinusoidal carrier; narrowband (NB): multiple closely spaced tones; wideband (WB): flat-spectrum Gaussian-like noise; chirp/swept: linear frequency-modulated sweep (the dominant PPD type); pulsed: distance measuring equipment (DME) and TACAN aeronautical radionavigation signals that occupy the L5/E5a band.

Core terminology defined: C/N0 is the carrier-to-noise density ratio (dB-Hz); J/N is the jammer-to-noise power spectral density ratio (dB); JSR is the jammer-to-signal power ratio (dB); BOC and CBOC refer to binary offset carrier and composite binary offset carrier modulations used in Galileo E1 and GPS L1C.

Standard references for this section: Kaplan and Hegarty (2017), Dovis (2015), Ioannides et al. (2016), Borio et al. (2016), and Groves (2013).

---

## 4. Findings

### 4.1 T1: Threat Landscape — Jammer Taxonomy and Real-World Incidents

**Evidence strength: Strong** | 10 sources | Levels II–VII

The foundational empirical survey by Mitch et al. (2011), downgraded to Level III by the verification report because the study characterises jammer emissions rather than testing receiver responses, established that all 18 sampled commercial devices employed linear frequency-modulation (chirp) with inter-device variability in sweep rate, bandwidth, and power. Effective jamming ranges exceeded manufacturer specifications by factors of 2–5. This variability bounds the generalisability of any fixed-parameter mitigation algorithm designed around a specific PPD profile, a limitation that converges with Borio et al. (2016), who show that chirp jammers dominate the commercial threat while wideband Gaussian sources produce the worst-case noise-floor elevation. The 2011 characterisation remains the sole systematic open dataset of commercial PPD parameters; the same group's later signal-tracking and geolocation study (Mitch et al., 2016) addresses receiver-side detection of chirp jammers but does not refresh the commercial-device parameter survey, and post-2015 designs with higher sweep rates and multi-band capability have not been systematically characterised in the open literature.

The global threat picture shifted substantially with Murrian et al. (2021), a three-year LEO monitoring campaign that geolocated a Syrian jamming source to sub-kilometre accuracy and documented recurrent jamming zones in the Eastern Mediterranean, Baltic, and Black Sea from ISS orbit. Felux et al. (2024), whose correct authorship (Felux, Fol, Figuet, Waltert, Olive) was confirmed via source verification after an author-substitution error was detected and corrected in the original bibliography, independently corroborated those geographic zones using crowd-sourced ADS-B data for 2022, showing that large commercial aircraft still experienced nuisance warnings and procedure unavailability even with inertial backup present. The convergence between Doppler-from-orbit geolocation and crowd-sourced ADS-B correlation across two methodologically independent techniques is notable evidence that the same geographic jamming sources produce measurable effects at operational aircraft altitudes.

The Osechas et al. (2022) instrumented A320 test flight provides the link between detected interference and actual avionics failure modes: commercial multimode receivers experienced full GNSS outages confirmed by pre-correlation RF spectral recording, with no C/N0 transparency to flight crew. At the infrastructure level, Pullen and Gao's (2012) Newark GBAS case remains the canonical example of how a single low-cost PPD can produce repeated disruption of safety-critical ground infrastructure , a finding that has motivated every subsequent aviation anti-jamming survey (Ioannides et al., 2016; Morales-Ferre et al., 2020).

The threat taxonomy assembled by Dovis (2015) and the Proceedings of the IEEE 2016 GNSS special issue (Borio et al., 2016; Ioannides et al., 2016) covers CW, narrowband, wideband, chirp, and pulsed jammer classes with analytical degradation models. The field has not produced a post-2015 systematic PPD characterisation campaign, leaving the Mitch et al. (2011) baseline as the only open-literature anchor for mitigation algorithm design, a primary empirical gap that directly affects the realism of all subsequent simulation-based technique evaluations.

---

### 4.2 T2: Spatial / Antenna-Array Mitigation

**Evidence strength: Strong** | 10 sources | Levels III–VII

Spatial processing is the most technically mature mitigation theme in the corpus, with a coherent evidence chain from analytical foundations through hardware demonstration. The Brennan (1959) combining theory and Fante and Vaccaro (2000) STAP derivations establish the theoretical performance ceiling; the post-2015 experimental literature consistently approaches but never cleanly reaches it due to calibration imperfections.

Three groups reporting independently in the same IEEE special issue, namely Fernández-Prades et al. (2016), Broumandan et al. (2016), and Cuntz et al. (2016), all find that minimum variance distortionless response (MVDR) class beamformers achieve null depths exceeding 40 dB under controlled HIL conditions, and all identify phase/amplitude calibration as the binding constraint on real-world performance. This convergence is architecturally important: it localises the gap between laboratory and deployed CRPA performance to an engineering and manufacturing problem rather than a fundamental signal-processing barrier. Fernández-Prades et al. (2016) provide the strongest experimental anchor at Level III, demonstrating a GNU Radio prototype with real RF data, and their open-source GNSS-SDR framework enables reproducible baselines for subsequent work.

Cuntz et al. (2016) extend the architecture by embedding CRPA null-steering within a vector-tracking loop, demonstrating at HIL level that the combination improves performance under both jamming and signal-blockage scenarios. The joint architecture breaks the historical separation between RF and baseband mitigation stages: the receiver processing chain functions as a coupled system, not a cascade of independent modules. Broumandan et al. (2016) add eigenspace decomposition for multi-jammer scenarios, identifying covariance estimation latency as a structural constraint for rapidly moving sources.

Daneshmand et al. (2016) contribute a Level III result that is analytically important but often overlooked: blind beamforming introduces carrier-phase bias that is only partially absorbed by the receiver clock state. For real-time kinematic (RTK) and precise point positioning (PPP) applications at centimetre level, the differential residual is not negligible. This finding bridges directly to the ANF/RIM bias discussion in T4.

The sparse array sub-theme (Amin et al., 2016; Zhang et al., 2016) addresses the size, weight, and power (SWaP) constraint on full CRPAs. Nested and coprime arrays achieve virtual apertures of order N² for N physical elements analytically, with simulation-demonstrated direction-of-arrival (DoA) accuracy superior to uniform linear arrays. However, this sub-theme sits entirely at Level V, with no hardware prototype in the corpus and mutual coupling characterisation for non-uniform geometries unresolved. DL-based beamforming (Ramezanpour et al., 2020a, 2020b) replaces covariance-inversion weight computation with CNN and Bi-LSTM architectures, reporting 10 dB SINR improvement over classical beamformers at 100 snapshots in simulation. Both papers are simulation-only at Level V, and neither benchmarks inference compute cost on embedded or FPGA hardware; these results require cross-validation from non-JRC, non-Mosavi-group sources before field deployment claims can be made.

---

### 4.3 T3: Pre-Correlation Excision — Pulse Blanking, FDAF, and Robust ZMNL

**Evidence strength: Moderate–Strong** | 9 sources | Levels II–VII

The pre-correlation excision literature occupies a structurally pivotal position: techniques here operate before GNSS code correlation, so their imperfections propagate into every downstream stage. Three sub-streams converge: pulse blanking for chirp and pulsed interferers, FDAF for narrowband tonal interference (particularly DME/TACAN in the L5/E5a band), and the unified RIM/ZMNL framework.

The strongest field-validated evidence is Gao et al. (2013) at Level II, from a real flight test over Frankfurt's dense DME/TACAN airspace demonstrating that hybrid blanking plus FDAF recovers C/N0 to within 2 dB of an interference-free reference for L5/E5a. This result grounds the analytical blanking chain: the Bastide et al. (2004) Poisson pulse-collision model predicts up to 10 dB C/N0 degradation in high-density European airspace; Garcia-Peña et al. (2021) extended the model analytically to pulsed duty cycles above 20%, correcting inaccuracies in the RTCA DO-292 baseline; Kim et al. (2025) close the chain by characterising how L5/E5a spreading code correlation properties degrade under blanking duty cycles from 0% to 75%. The mechanism chain (Bastide → Garcia-Peña → Kim) is analytically coherent, with the Gao et al. (2013) flight test providing the empirical anchor at the most operationally important operating point.

Borio's (2016) pulse blanking analysis for chirp PPDs establishes that swept jammers, once band-limited by the receiver front-end, become pulsed interference amenable to time-domain blanking, achieving 10–15 dB J/N improvement for typical PPD parameters in simulation-supported analytical evaluation (single-author Level V; the figure is derived from the duty-cycle model and has not been independently re-measured in HIL or field within the corpus). The physical mechanism (band-limiting converts a continuous chirp sweep into a duty-cycled pulse train) connects PPD mitigation directly to the DME blanking literature through the same mathematical framework. Borio and Closas (2019) then extend this unification: ZMNL techniques (signum, Huber, Student-t) applied after DFT or wavelet transforms provide excision without requiring jammer type identification, maintaining acquisition sensitivity within 2 dB of an interference-free reference at J/N ≤ 30 dB in hardware-simulator testing. The practitioner summary by Borio and Closas (2017) explicitly maps signum ZMNL as reducing pulsed interference to approximately 3 dB effective noise elevation versus 20–30 dB unmitigated, providing the most accessible cross-audience statement of the technique's practical value.

The wavelet excision sub-stream (Musumeci and Dovis, 2014; Mosavi et al., 2017) demonstrates at Level V that wavelet packet decomposition (WPT) effectively separates interference across CW, narrowband chirp, and pulsed scenarios. Dual-stage WPT architecture provides approximately 15 dB additional rejection over single-stage, with acquisition probability above 90% at J/N = 45 dB for CW in simulation. These results are internally consistent but lack HIL or field validation; wavelet basis selection and threshold adaptation under varying interference power remain open engineering problems.

Kim et al. (2025) provide an important planning constraint: the advantage of L5/E5a over L1 C/A in terms of correlation peak quality is preserved up to approximately 60% blanking duty cycle. Above that threshold, L5 correlation degradation reaches L1 C/A parity, eliminating the signal-quality advantage of L5 in exactly the high-density European DME environments where it is most operationally needed.

---

### 4.4 T4: Baseband / Transform-Domain Mitigation — ANF, STFT, and RIM

**Evidence strength: Moderate** | 10 sources | Levels III–VII

The baseband mitigation literature is theoretically the most unified of any theme, owing to Borio and Closas's (2019) robust statistics framework that subsumes ANF, pulse blanking, and wavelet excision under a common ZMNL architecture. Yet this unification coexists with a concrete operational asymmetry: ANF introduces pseudorange and clock bias that RIM techniques do not.

Borio's (2016) ANF loop analysis establishes the theoretical equivalence between adaptive notch filter dynamics and frequency lock loop (FLL) theory. This equivalence is more than a mathematical curiosity: it provides engineers with loop-bandwidth and Doppler-bandwidth tools for ANF design, and it predicts the linear-approximation breakdown at J/N > 40 dB, a calibration point for when RIM should be preferred. Amin et al. (2017) show that short-time Fourier transform (STFT)-based excision outperforms ANF for chirp and frequency-modulated continuous-wave (FMCW) interference specifically, because the STFT resolves the sweep's instantaneous frequency in the time-frequency (TF) plane whereas a notch filter tracks a single carrier. Quadratic time-frequency distributions (TFDs) offer superior TF resolution but introduce cross-term artefacts in adjacent bins, creating a failure mode that can corrupt nearby GNSS signal energy.

The critical experimental contribution in this theme is Borio and Gioia (2021), providing at Level III the most direct comparison (real jammer IQ captured via a USRP2 hardware chain, processed through a custom MATLAB software receiver with real GPS, Galileo, and BeiDou signals) of ANF against RIM within a single experimental setup. ANF was found to introduce a common-mode delay bias of 1–5 m under strong jamming, while RIM techniques (signum and Huber ZMNL) introduced no pseudorange bias within that setup. The 1–5 m envelope must be read as a setup-specific bound measured under one front-end, one ADC quantisation, one ANF bandwidth setting, and one jammer-power profile; independent replication on a different hardware chain has not appeared in the corpus. It must further be noted that the ANF-versus-RIM comparison framing originates from the same JRC research group (Borio and collaborators) that developed the unifying RIM/ZMNL framework, creating a structural risk that the comparison foregrounds conditions favourable to RIM; a methodologically symmetric study from a non-JRC, non-RIM-developing group would substantially strengthen the conclusion.

The "common-mode" designation is important for interpreting positioning consequences. A bias identical across all pseudoranges is absorbed by the receiver clock state in single-constellation single-point positioning and does not degrade horizontal position accuracy. However, a differential residual remains in multi-constellation processing where ANF bandwidth affects signal types differently, and the bias is not common-mode across frequencies in dual-frequency ionospheric-correction combinations. For high-precision applications (RTK, PPP) and for timing receivers (where ANF's clock bias effect was independently quantified by Borio and Gioia (2021, GPS Solutions) within the same JRC research group), ANF is disqualified as the sole mitigation technique when J/N exceeds approximately 30 dB *within the Borio and Gioia experimental envelope*; extrapolation to other receiver architectures requires explicit re-characterisation.

Multi-constellation processing (GPS + Galileo + BeiDou) under RIM mitigation approximately halved positioning error relative to GPS-only within the Borio and Gioia (2021) HIL experimental setup. Gioia and Borio (2023) report a directionally consistent benefit in a multi-layer cascade context, but this also originates from JRC authors, providing consistency rather than independent replication. Jafarnia-Jahromi et al. (2015) independently show that L1C/E1 CBOC provides a 3–5 dB advantage over L1 C/A for CW interference, mechanistically consistent with the *direction* of a multi-constellation diversity benefit but not a replication of the *magnitude* of joint multi-constellation positioning gain. This synthesis carries the directional claim (multi-constellation reduces positioning error under jamming) at acceptable evidence strength, while explicitly demoting the specific "halving" magnitude to a single-group, JRC-sourced HIL reference point requiring independent non-JRC confirmation before it can serve as a deployment specification.

---

### 4.5 T5: Tracking-Loop and Navigation-Filter Resilience

**Evidence strength: Moderate** | 9 sources | Levels II–VII

The tracking and navigation filter theme covers the widest architectural range in the corpus, from individual correlator bandwidth adaptation through vector tracking to tightly and deeply coupled GNSS/INS. Its central contribution is the recognition that resilience is cumulative: gains at each stage are potentially multiplicative when properly integrated, but interaction effects between stages can also produce unexpected failure cascades.

Cortés et al. (2021) and Cortés et al. (2023) demonstrate at Level V (simulation, MDPI Sensors venue) that adaptive loop-bandwidth control (LBCA) reduces phase tracking error by 30–50% under moderate interference relative to a standard 15 Hz PLL bandwidth, and that a low-complexity DSKF maintains lock 6 dB longer than a classical PLL at J/N = 30 dB. Both results are simulation-only; the verification report notes MDPI editorial process variability, and neither paper has tested the techniques against real jammer signals.

The most-cited vector-tracking quantitative anchor is Lashley and Bevly (2010), reporting a 3–5 dB J/N advantage of vector over scalar tracking and a further 5–8 dB advantage for deep-coupled MEMS IMU integration. This is a 2010 HIL-architecture result that has not been independently re-measured in the post-2015 corpus on modern receiver hardware against modern jammer classes. These figures should be carried as *legacy reference points* whose order of magnitude is theoretically defensible from the Bayesian inter-channel coupling framework (Section 9 of the synthesis derives this from Kalman filter optimality) but whose precise values under contemporary implementations have not been re-verified within the search window. Cortés et al. (2023) achieve a comparable 6 dB benefit through a mechanistically distinct route (DSKF inside a single channel), providing directional cross-corroboration for "single-digit dB" gains from advanced tracking architectures without constituting a replication of the Lashley and Bevly (2010) vector-tracking measurement specifically.

Deep GNSS/INS coupling evidence becomes most architecturally specific with Lashley and Bevly (2015) at Level III (real MEMS IMU hardware): MEMS-grade inertial deep coupling cannot bridge a 35-second continuous jamming window, with positioning error growing at approximately 1–2 m/s drift rate. The 35-second figure is the most-cited MEMS bridging-time limit in the corpus and is intrinsic to MEMS sensor physics: gyro drift and accelerometer bias grow with time, producing position uncertainty that compounds under double integration. The mitigation architecture context matters: when CRPA null-steering is co-present and reduces the effective J/N at the antenna by 30–40 dB, the inertial bridging subsystem may never activate for moderate jamming scenarios. For high-power wideband sources that overwhelm spatial mitigation (scenarios documented by Murrian et al. (2021) and Osechas et al. (2022)), the INS bridges an outage of indeterminate duration, and re-acquisition time after cessation is under-characterised across the corpus.

Gioia and Borio (2023) demonstrate at Level III that cascaded RIM + FDE + RAIM layers maintain positioning accuracy when any individual layer fails, with GPS plus Galileo dual-constellation contributing an approximate halving of positioning error versus GPS-only within their HIL SDR experimental envelope. This is a Level III, single-JRC-group result; the directional finding (cascaded layers outperform any single layer; multi-constellation benefits positioning under interference) is credible and corroborated by Ioannides et al. (2016) as a long-term resilience recommendation, but the specific magnitude is a single-group setup-specific reference point pending independent non-JRC field-trial replication.

Morales-Ferre et al. (2020) provide the meta-level observation for this theme: no single technique is sufficient across the full civilian threat spectrum, and INS-based approaches are most effective for high-power wideband jamming while spatial processing is most effective for directional single-source jamming.

---

### 4.6 T6: ML/DL Cross-Cutting — Classification, DL Beamforming, and the Simulation-to-Reality Gap

**Evidence strength: Emerging** | 8 sources | Levels V–VII

ML and DL constitute the fastest-growing area in the corpus by publication date and the weakest by evidence tier. Every classification and DL beamforming paper in the corpus sits at Level V or VII, all simulation-only, with no field-validated or HIL-validated performance figures.

Morales-Ferre et al. (2019) demonstrate that SVM achieves 94.9% and CNN achieves 91.4% classification accuracy across six jammer classes using simulation-derived time-frequency images. Swinney and Woods (2021) apply VGG16 transfer learning to a simulation-generated dataset, reporting high classification accuracy with concatenated signal representations (power spectral density, spectrogram, constellation, histogram). Both papers release open-access Zenodo datasets, enabling reproducibility within the simulation paradigm. The 91–95% accuracy figures imply a 5–9% misclassification rate, meaning one incorrect technique selection per 10–20 jammer encounters, which in a safety-critical setting means a misclassified wideband jammer (triggering ANF instead of blanking) could cause tracking loss where mitigation would otherwise succeed. This failure operates silently; the receiver applies a technique without reporting its classification decision.

Radoš et al. (2024) provide the clearest meta-statement of the theme's state: ML detectors outperform threshold-based methods in controlled settings (>95% accuracy) but "the simulation-to-reality deployment gap is the field's primary open problem," and open-set classification (generalising to jammer types not represented in training) is unsolved. In the real operational environment, this is not a hypothetical concern: commercial jammer technology evolves continuously, and improvised software-defined radio (SDR)-based jammers can combine characteristics of multiple classes in ways not anticipated by fixed training datasets.

DL beamforming (Ramezanpour et al., 2020a, using CNN; Ramezanpour et al., 2020b, using Bi-LSTM plus CNN) reports 10 dB SINR improvement over classical beamformers at 100 snapshots, with ISR up to 34 dB, in simulation. Neither paper benchmarks inference latency on embedded FPGA or SoC hardware. Whether a Bi-LSTM forward pass completes within the GNSS correlation interval at 50 Hz tracking rates is an unaddressed engineering question. The Mosavi and Shafiee (2016) early ML paper reports a Sigma-Pi neural network improving residual CW suppression by approximately 5 dB beyond cascaded ANF in simulation. This represents the earliest ML entry in this theme and is also simulation-only.

The mechanism by which DL outperforms classical techniques is theoretically plausible: neural networks can learn non-linear interference-signal boundaries that fixed ZMNLs cannot represent. But this advantage materialises only when the inference-time jammer is drawn from the same distribution as the training set. The distribution-shift problem, the central challenge of deploying ML in operational environments, is present in every ML paper in this theme and resolved in none.

---

## 5. Evidence Map

### 5.1 Framing

The evidence map below renders the Technique Stage × Jammer Type × Metric dimension as a matrix that serves as the central analytical contribution of this synthesis. Cell coding follows the five-level scheme described in Section 2.5. The map reveals structural patterns not visible from individual paper reviews: spatial mitigation evidence concentrates in CW and NB columns with a systematic gap in pulsed (DME) rows; pre-correlation excision is the only stage with field-level evidence for chirp and pulsed jammers; ML has no HIL or field evidence in any cell.

**Table 1**
*Evidence Map: Technique Stage × Jammer Type (as of May 2026)*

*Note.* CW = continuous wave; NB = narrowband multi-tone; WB = wideband Gaussian; Chirp = swept FM (PPD); Pulsed = DME/TACAN or blanked chirp. Cell codes: **Strong** = field trial or HIL, multiple independent groups; **Moderate** = HIL/lab, ≥2 sources, ≥2 groups; **Single-group** = HIL/lab from one group only; **Sim** = simulation only; **Gap** = no source in corpus.

*Footnote a.* CRPA MVDR cells are coded **Moderate** (not Strong) because the three reporting groups (Fernández-Prades et al., 2016; Broumandan et al., 2016; Cuntz et al., 2016) all published in the same coordinated *Proceedings of the IEEE* June 2016 GNSS special issue, reducing methodological independence between the reported results. No post-2016 field-level independent CRPA MVDR replication appears in the corpus that would support a Strong rating.

| Technique Stage | Metric | CW | NB | WB | Chirp | Pulsed (DME) |
|---|---|---|---|---|---|---|
| RF/Antenna (CRPA MVDR) | Null depth, SINR | Moderate | Moderate | Moderate | Moderate | Gap |
| RF/Antenna (sparse array) | SINR, DoA | Sim | Sim | Gap | Sim | Gap |
| RF/Antenna (DL beamforming) | SINR | Sim | Sim | Sim | Sim | Gap |
| ADC/Pre-corr (pulse blanking) | C/N0, acq. prob. | Gap | Gap | Gap | Strong | Strong |
| ADC/Pre-corr (FDAF) | C/N0 | Gap | Moderate | Gap | Gap | Moderate |
| Baseband (ANF) | C/N0, psrange bias | Strong | Moderate | Gap | Moderate | Gap |
| Baseband (RIM/ZMNL signum) | C/N0, position err | Moderate | Moderate | Moderate | Moderate | Moderate |
| Baseband (wavelet excision) | Acq. prob. | Sim | Sim | Gap | Sim | Gap |
| Baseband (TF/STFT excision) | SINR, classif. | Gap | Gap | Gap | Moderate | Gap |
| Tracking (adaptive bandwidth) | Phase tracking err | Sim | Sim | Sim | Gap | Gap |
| Tracking (vector tracking) | JSR, J/N | Moderate | Gap | Moderate | Gap | Gap |
| Navigation (INS deep coupling) | Position err | Gap | Gap | Moderate | Gap | Gap |
| Navigation (multi-layer FDE+RAIM) | Position err | Moderate | Moderate | Moderate | Moderate | Gap |
| ML classification | Classif. accuracy | Sim | Sim | Sim | Sim | Gap |
| Multi-constellation (passive) | Position err | Single-group | Single-group | Single-group | Single-group | Gap |

*Sources: synthesis derived from Brennan (1959); Fante & Vaccaro (2000); Fernández-Prades et al. (2016); Broumandan et al. (2016); Cuntz et al. (2016); Amin et al. (2016); Zhang et al. (2016); Ramezanpour et al. (2020a, 2020b); Gao et al. (2013); Bastide et al. (2004); Garcia-Peña et al. (2021); Kim et al. (2025); Borio (2016, blanking); Borio (2016, ANF); Borio & Closas (2017, 2019); Borio & Gioia (2021, NAVIGATION; 2021, GPS Solutions); Amin et al. (2017); Musumeci & Dovis (2014); Mosavi et al. (2017); Cortés et al. (2021, 2023); Lashley & Bevly (2010, 2015); Gioia & Borio (2023); Morales-Ferre et al. (2019, 2020); Swinney & Woods (2021); Radoš et al. (2024); Jafarnia-Jahromi et al. (2015); Mosavi & Shafiee (2016).*

### 5.2 Key Gaps Identified

Five structural gaps stand out from the map:

1. **Pulsed (DME) interference against spatial processing.** The entire CRPA row for the Pulsed column is a Gap. Whether MVDR null-steering provides useful rejection against the broadband spectral signature of dense DME environments has not been tested at HIL or field level.

2. **Wideband jamming against wavelet and STFT excision.** These transform-domain techniques have no WB column entries at or above simulation level for acquisition/tracking metrics.

3. **Chirp/swept jamming against vector tracking.** The vector tracking row has no Chirp column entry, yet chirp PPDs are the dominant real-world threat category.

4. **Any technique against high-duty-cycle (>60%) DME in field conditions beyond Gao et al. (2013).** The one flight-test result is over a decade old; European airspace DME density has evolved.

5. **ML classification or DL beamforming against real jammer signals in any hardware trial.** The entire ML row has no entry above Sim.

---

## 6. Failure-Mode Synthesis

Provenance tags (*[Observed-incident]*, *[HIL-demonstrated]*, *[Theory-predicted]*) categorise the evidence basis for each failure mode. The tags do not rank operational severity: a theory-predicted failure (e.g., open-set ML misclassification) may pose greater operational risk than an observed-incident mode with known mitigations.

### 6.1 Front-End Saturation: The Upstream Wall — *[Observed-incident + Theory-predicted]*

All baseband mitigation shares a common upstream failure mode: AGC nonlinearity and ADC clipping under extreme J/N. When jammer power exceeds AGC dynamic range (approximately 40–50 dB above thermal noise in civilian front-ends; Kaplan & Hegarty, 2017), the AGC compresses both the jammer and the desired signal, and the ADC clips both. Borio and Gioia (2021) note explicitly that high J/N jamming can saturate the front-end before RIM becomes effective, and the companion timing study (Borio & Gioia, 2021, *GPS Solutions*) confirms that even multi-constellation timing receivers require oscillator holdover under very high J/N. Front-end saturation establishes a hard physical ceiling: baseband mitigation is bounded above by front-end headroom, and no algorithmic sophistication can overcome it without hardware changes.

### 6.2 Calibration and Phase Coherence — *[HIL-demonstrated]*

For spatial processing, the binding real-world constraint is phase and amplitude calibration. Calibration errors arising from mutual coupling, cable mismatches, thermal drift, and installation geometry deviation reduce achievable null depth from the theoretical 40+ dB to approximately 20–25 dB in practice (Fernández-Prades et al., 2016; Cuntz et al., 2016). Daneshmand et al. (2016) add that blind beamforming introduces carrier-phase bias that propagates to positioning accuracy even when null depth is maintained. Sparse arrays (Amin et al., 2016) face an additional unresolved problem: mutual coupling characterisation in non-uniform geometries. Calibration failure is gradual and may remain invisible to the receiver's internal diagnostics while steadily eroding interference rejection.

### 6.3 Threshold Dependency — *[HIL-demonstrated + Theory-predicted]*

Both pulse blanking and ANF require parameter settings that balance interference rejection against desired signal preservation. For blanking, the Garcia-Peña et al. (2021) model captures this trade-off explicitly, and Kim et al. (2025) quantify the consequence for L5/E5a: above 60% duty cycle, L5 correlation quality reaches L1 C/A parity, removing L5's advantage precisely in the high-density DME environments where it is most needed. For ANF, Borio (2016) shows that the linear design approximation breaks down at J/N > 40 dB. In operational scenarios, jammer power is unknown and time-varying; a PPD in a passing vehicle produces a time-varying J/N sweep requiring adaptive threshold management that no corpus paper demonstrates in real-time embedded implementation.

### 6.4 ML/DL Simulation-Only Failure Modes — *[Theory-predicted]*

The ML theme is the most extreme case of simulation-only validation in the corpus. Failure modes invisible to simulation include: (a) AGC compression and ADC quantisation noise altering the feature statistics used for classification; (b) multipath-induced time-varying amplitude and phase modulation at the antenna, absent from channel-free simulation; (c) front-end hardware variability across receiver models producing different intermediate-frequency (IF) waveforms from the same jammer. Transfer learning from ImageNet (Swinney & Woods, 2021) introduces a domain mismatch, since RF interference spectrograms differ fundamentally from natural image statistics, requiring empirical domain-adaptation validation not yet present in the corpus. None of these failure modes can be characterised without real hardware integration.

### 6.5 INS Bridging-Time Limits — *[HIL-demonstrated]*

Lashley and Bevly (2015) establish that MEMS-grade IMUs accumulate position error at approximately 1–2 m/s during GNSS outages, bounding the utility of deep coupling to jamming durations below approximately 10–35 seconds depending on application accuracy requirements. This limit is intrinsic to MEMS sensor physics. In a combined mitigation architecture where CRPA null-steering reduces effective J/N at the RF stage, inertial bridging may never activate for moderate jamming. For high-power wideband jamming that overwhelms spatial mitigation, as documented by Murrian et al. (2021) and Osechas et al. (2022), the INS must bridge an outage of unknown duration, and re-acquisition time after cessation is under-characterised, particularly for multi-constellation receivers where simultaneous re-acquisition may interact with tracking loop dynamics.

### 6.6 Open-Set Classification Failure — *[Theory-predicted]*

All ML classification systems in the corpus operate on closed-set assumptions: a fixed number of training classes corresponding to known jammer types. Radoš et al. (2024) identify open-set classification as unsolved. In the real threat environment, this is operationally significant: a novel commercial device or improvised SDR-based jammer combining characteristics of multiple classes will be assigned to the nearest training class with no mechanism for flagging out-of-distribution input. The downstream consequence is a confident but incorrect mitigation selection; for example, applying ANF (appropriate for CW) to a wideband source, providing no benefit while consuming processing resources. The structural fix of open-set recognition with a rejection class and calibrated confidence scoring is established in the general ML literature but has not been demonstrated in the GNSS interference context.

### 6.7 ANF Common-Mode Delay Bias for High-Precision PNT — *[HIL-demonstrated, single-group]*

The ANF bias failure mode warrants separate treatment because it is quantified (1–5 m under strong jamming within the Borio and Gioia (2021) USRP2 + MATLAB experimental envelope, a setup-specific bound and not a universal ANF property) yet operates silently. The common-mode nature means single-frequency single-constellation receivers absorb most of the bias in the clock state, reporting apparently reasonable position accuracy while carrying a systematic offset. The failure activates specifically under: (a) dual-frequency ionospheric correction, since bias magnitude differs by frequency; (b) differential positioning, where receiver-specific bias differences corrupt baselines; (c) timing infrastructure using absolute pseudorange for clock synchronisation. Railway signalling, maritime precision approach, and aviation precision landing are precisely the use cases where ANF is appealing as a low-cost baseband mitigation yet where this silent bias causes the greatest harm. The resolution is clear: use RIM (signum or Huber ZMNL) rather than ANF for any application requiring pseudorange accuracy better than a few metres under moderate-to-strong jamming. This conclusion originates from the same JRC group that developed RIM; independent replication would substantially strengthen it.

---

## 7. Discussion

### 7.1 Cross-Theme Integration: The Receiver Chain as a Coupled System

The most important architectural insight from this synthesis is that the receiver chain is a coupled system, not a cascade of independent modules. The evidence chain runs from Cuntz et al. (2016), who combined CRPA null-steering with vector tracking at the HIL level, through Gioia and Borio (2023), who demonstrated that cascaded RIM plus FDE plus RAIM outperforms any single layer, to the theoretical integration in Lashley and Bevly (2010), who showed that the Bayesian state-space formulation of vector tracking expresses inter-channel coupling mathematically.

The practical implication is a cumulative-gain principle: spatial processing at the antenna reduces J/N by 30–40 dB for directional jammers; pre-correlation excision handles residual pulsed and chirp energy; RIM addresses narrowband residuals at the correlator; vector tracking maintains lock through inter-channel state sharing; INS aiding bridges brief outages. Each stage extends the composite architecture's operating range. No study in the corpus demonstrates this full architecture against high-power wideband sources at field level; this constitutes the most operationally consequential experimental gap.

Stage interactions are not always beneficial. RIM-induced distortion propagates through FDE and RAIM in ways not yet modelled theoretically (Gioia & Borio, 2023). ANF bias, if present before position-domain FDE, corrupts the fault-detection residuals. Vector tracking's inter-channel coupling can propagate corrupted data from a partially jammed channel to all others, the known contamination failure mode of vector receivers.

### 7.2 Contradiction Resolutions

**ANF pseudorange bias.** Early treatments (Dovis, 2015; Borio, 2016) characterised C/N0 recovery as the primary ANF metric, implicitly treating the technique as effectively bias-free. Borio and Gioia (2021) extended measurement to J/N = 35 dB using a real jammer, where the 1–5 m bias becomes positioning-relevant. These results are reconcilable by test-condition scope: at J/N ≤ 20 dB in single-point positioning, the bias is absorbed by the clock state and is below positioning-relevant thresholds; at higher J/N in precision applications, it is not. The boundary condition is receiver-architecture-specific and requires explicit characterisation for each deployment context.

**ML simulation vs. real-world accuracy.** Morales-Ferre et al. (2019) and Swinney and Woods (2021) report 91–95% classification accuracy on simulation data; Radoš et al. (2024) identify the simulation-to-reality deployment gap as the field's primary open problem. These claims are not contradictory; they describe different evaluation domains. Simulation accuracy is valid within its domain; treating it as a deployment proxy requires domain-shift assumptions that none of the corpus papers validates. Resolution: simulation accuracy should be reported as a conditional upper bound; gap size is currently unknown and constitutes the most actionable open question in the ML theme.

**MEMS INS as bridging vs. resilience pillar.** Lashley and Bevly (2015) establish that MEMS INS cannot bridge a 35-second continuous outage. Cuntz et al. (2016), Gioia and Borio (2023), and Felux et al. (2024) treat GNSS/INS integration as a primary resilience measure. These positions are reconcilable by functional distinction: MEMS cannot replace GNSS entirely for sustained high-power jamming (the Lashley 2015 result), but in a combined architecture where spatial mitigation reduces J/N at the RF stage, the INS aiding role shifts from standalone backup to state-continuity augmentation during brief signal degradations. The literature conflates these two use cases; the distinction must be explicit in any deployment specification.

**Multi-constellation magnitude claim.** Borio and Gioia (2021) report approximately halved positioning error for GPS plus Galileo plus BeiDou versus GPS-only within their single-group JRC HIL setup. Gioia and Borio (2023) provide directionally consistent corroboration within the same group. Ioannides et al. (2016) recommend multi-constellation as a long-term resilience strategy without independent quantification. The direction of effect is supported by mechanism (satellite geometry, FDE redundancy, signal diversity) and by the JRC measurements; the specific "halving" magnitude is a single-group setup-specific reference point pending non-JRC independent replication.

**L5/E5a blanking duty-cycle threshold.** Bastide et al. (2004) and Garcia-Peña et al. (2021) characterise C/N0 degradation from DME interference; Kim et al. (2025) characterise correlation-domain distortion from the blanking operation itself. These describe complementary phenomena: blanking removes interference (improving effective J/N) while simultaneously distorting the desired signal (the Kim finding). At low duty cycles, interference removal dominates. Above approximately 60%, blanking distortion closes the L5 versus L1 C/A quality gap. The two result sets are not contradictory but describe different physical mechanisms at the same operating point.

**Sparse array theory vs. deployment reality.** Amin et al. (2016) and Zhang et al. (2016) analytically demonstrate N² virtual aperture advantages; no hardware prototype appears in the corpus. This is not an internal contradiction but a theory-to-deployment gap: the theoretical advantage is well-established, but mutual coupling characterisation and field validation are absent. Sparse array performance claims should be treated as theoretical upper bounds until hardware demonstration is available.

**Vector tracking legacy reference.** Lashley and Bevly (2010) report 3–5 dB vector-tracking advantage and 5–8 dB deep-coupling gain in a 2010 HIL architecture study. No post-2015 paper independently replicates these specific figures on modern hardware against modern jammer classes. Cortés et al. (2023) achieve a 6 dB benefit via a mechanistically distinct DSKF approach, providing directional cross-corroboration for single-digit dB gains without replicating the specific measurement.

### 7.3 State-of-the-Art Receiver-Side Architecture for 2026

Effective receiver-side anti-jamming architecture is not a single technique but a platform-differentiated stack matched to SWaP constraints and application accuracy requirements.

**Handheld and consumer-grade receivers** have no space for a multi-element antenna. Available mitigations are limited to baseband: RIM (signum ZMNL) provides the highest interference rejection without pseudorange bias, and multi-constellation reception provides passive diversity gain. ANF should not be used as the sole mitigation for any application requiring sub-metre accuracy.

**Vehicular receivers** can accommodate multi-frequency (L1/L5 or L1/E1/E5a) reception. The optimal architecture adds adaptive loop bandwidth control (LBCA or DSKF) to baseband RIM, with MEMS INS tight coupling for bridging. L5/E5a blanking duty-cycle limits (Kim et al., 2025) should be monitored near dense DME corridors. Multi-constellation RIM with FDE provides the strongest civilian-platform anti-jamming capability within SWaP constraints.

**Avionics receivers** (aircraft navigation, GBAS, SBAS landing guidance) are the highest-consequence application. Felux et al. (2024) and Osechas et al. (2022) make a direct case for multi-layer cascade: CRPA null-steering at the antenna, hybrid blanking plus FDAF for L5/E5a DME environments (demonstrated by Gao et al., 2013), RIM at the correlator, vector tracking, and tightly coupled INS for bridging. For precision approach in particular, the Borio and Gioia (2021) ANF bias finding argues against ANF as the sole baseband mitigation within its experimental envelope; whether the same envelope generalises to certified avionics front-ends remains an open characterisation question.

**Timing receivers** (telecom synchronisation, financial infrastructure, power grid) are most sensitive to the ANF common-mode delay bias documented by Borio and Gioia (2021, *GPS Solutions*). Timing receivers in known jamming regions should prioritise RIM over ANF and implement multi-constellation holdover.

### 7.4 Limitations of This Review

This review has four categories of limitation that practitioners should weigh when applying its findings.

*Corpus composition.* The English-only restriction excludes active Chinese-language BeiDou anti-jamming research and German-language DLR contributions appearing only in German. JRC institutional concentration at approximately 16% of the corpus (entries #9, #24, #25, #27, #30, #31, #32, #42, #43; Borio, Gioia, Garcia-Peña) creates a corroboration risk for JRC-sourced quantitative claims, most critically the ANF 1–5 m bias envelope and the multi-constellation 50% error reduction magnitude, both of which this synthesis has explicitly demoted to single-group reference points. DLR institutional concentration (entries #17, #26; Gao and team) is lower severity given the Gao et al. (2013) flight-test external anchor.

*Verification coverage.* Eighty-six percent of the corpus was not individually spot-checked beyond a structural audit. The Felux et al. (2024) author-substitution correction (the original bibliography attributed DOI 10.33012/navi.657 to entirely different individuals, "Štern, Džunda, Vosburgh", before the source verification agent caught and corrected the error) is a concrete illustration of residual author-attribution risk across the unspot-checked majority.

*ML theme evidence level.* The entire ML and DL theme is simulation-dominated. This synthesis characterises the state of the research programme, not the state of deployed technique performance. Treating the 91–95% simulation accuracy figures as deployment performance estimates would be a misreading of the evidence.

*Temporal cutoff.* The May 2026 search cutoff means the synthesis does not capture the most recent ML/DL publications in a domain advancing rapidly. The evidence map in Table 1 reflects the state of knowledge at the time of the search.

---

## 8. Conclusion and Recommendations

### 8.1 Answer to the Primary Research Question

Civilian GNSS receiver-side anti-jamming techniques evolved substantially between 2015 and 2026, progressing from single-technique demonstrations to architecturally integrated multi-layer systems combining spatial processing, pre-correlation excision, RIM-based baseband filtering, adaptive tracking, and INS aiding. Threat characterisation has been strengthened by independent LEO monitoring (Murrian et al., 2021), instrumented flight testing (Osechas et al., 2022), and large-scale crowd-sourced analysis (Felux et al., 2024), confirming that jamming is an ongoing operational condition. Demonstrated performance in HIL and field environments is strongest for CRPA null-steering (>40 dB null depth at HIL level), pre-correlation blanking plus FDAF (confirmed by the Gao et al., 2013 flight test), and RIM-based baseband excision (laboratory HIL single-group). The principal gap between research and deployment readiness is the absence of a full combined-mitigation architecture evaluation against high-power wideband sources at field level, and the simulation-only validation of the entire ML/DL sub-discipline.

### 8.2 Highest-Priority Knowledge Gaps

- **Post-2015 PPD signal characterisation.** The Mitch et al. (2011) 18-device campaign is the sole open characterisation of commercial jammer parameters. A new measurement campaign covering multi-band, higher-sweep-rate post-2015 designs is the highest-priority empirical gap.
- **Real-jammer ML training and validation datasets.** The simulation-to-reality gap cannot be closed without a curated, multi-receiver, labelled IQ dataset analogous to MNIST for computer vision. Dataset creation is a precondition for any field-valid ML claim.
- **Full combined-mitigation HIL evaluation against wideband sources.** No study in the corpus demonstrates CRPA plus RIM plus vector tracking plus INS against high-power wideband jamming at HIL or field level.
- **Multi-constellation active mitigation quantification from non-JRC groups.** The approximately halved positioning error under jamming reported by Borio and Gioia (2021) and Gioia and Borio (2023) is currently a single-group HIL reference point, not an independently confirmed deployment specification. Non-JRC field-trial or HIL replication is required before the specific magnitude (rather than the direction of effect) can serve as a deployment specification.
- **ML inference latency on embedded GNSS hardware.** Neither DL beamforming nor classification papers benchmark forward-pass latency on FPGA, SoC, or DSP platforms representative of deployed receivers.

### 8.3 Practitioner Recommendations

(a) Within the Borio and Gioia (2021) experimental envelope (USRP2 plus custom MATLAB receiver chain, real GPS/Galileo/BeiDou signals, J/N up to 35 dB), ANF introduced a 1–5 m pseudorange bias that RIM techniques did not. For high-precision PNT applications (RTK, PPP, timing), this finding argues against deploying ANF as the sole baseband mitigation above J/N ≈ 30 dB; RIM/ZMNL is the preferred alternative within that envelope. Extrapolation to other receiver platforms requires explicit re-characterisation rather than blanket adoption of either choice.

(b) For safety-critical avionics, deploy a multi-layer cascade with INS aiding rather than relying on any single mitigation stage. The 35-second MEMS bridging limit (Lashley & Bevly, 2015) means sustained high-power jamming overwhelms an INS-only fallback.

(c) Treat ML classification accuracy figures as upper bounds conditional on domain match to the training set. Do not specify system performance requirements from simulation-only figures.

(d) Treat multi-constellation magnitude claims (GPS plus Galileo plus BeiDou approximately halving positioning error) as single-group JRC HIL reference points pending independent field-trial replication, not as confirmed deployment specifications.

### 8.4 Future Research Directions

The highest-value directions are: (1) a coordinated campaign to characterise post-2015 commercial PPD signal parameters; (2) construction of an open, labelled real-jammer IQ dataset for ML training; (3) a full combined-mitigation HIL or field study against high-power wideband threats representative of the Eastern Mediterranean incidents; (4) independent non-JRC replication of the multi-constellation magnitude results; (5) FPGA/SoC latency benchmarking of DL-based mitigation; and (6) explicit modelling of cross-stage interaction effects (RIM-induced distortion propagating into FDE/RAIM residuals, vector-tracking inter-channel contamination from a partially jammed channel, ANF bias corruption of fault-detection statistics), all of which the current corpus identifies as plausible failure paths but none of which any single study isolates experimentally. Progress on these fronts would substantially close the gap between the research frontier and operational deployment readiness documented in this synthesis.

---

## 9. Declarations

### 9.1 Conflicts of Interest

The JRC (Joint Research Centre, EU Commission) research group (Borio, Gioia, Garcia-Peña) contributed approximately 16% of the corpus (bibliography entries #9, #24, #25, #27, #30, #31, #32, #42, #43). JRC researchers evaluate techniques they may recommend for EU standards; no financial conflict of interest is present, but the institutional concentration creates a corroboration risk for JRC-sourced quantitative claims, which have been explicitly flagged and demoted where independent replication is absent. The DLR (German Aerospace Centre) institutional concentration (entries #17, #26, Gao and team) is lower severity, mitigated by the external anchor provided by the Gao et al. (2013) flight-test data. The authors have no direct affiliations with either institution.

### 9.2 AI Use Disclosure

This research was conducted via the academic-research-skills v3.7.0 multi-agent pipeline (Claude Opus 4.7), with human direction at the scoping phase and three Devil's Advocate checkpoints (DA-CP1 at Phase 1, DA-CP2 at Phase 3, DA-CP3 at Phase 5). All citations were verified against original sources by the source_verification_agent; one critical author-substitution hallucination was caught and corrected during verification: the bibliography had attributed DOI 10.33012/navi.657 (Felux et al., 2024, *NAVIGATION: Journal of the ION*) to "Štern, Džunda, Vosburgh," entirely different individuals with no connection to that paper. The corrected authorship (Felux, Fol, Figuet, Waltert, Olive) was confirmed against the ION journal record and ResearchGate, and is used throughout this paper. This correction is surfaced as a worked example of why author-field verification cannot be substituted by DOI-only confirmation. Beyond this corrected instance, 86% of the 58-source corpus was not individually spot-checked for author-attribution beyond structural audit; residual hallucination risk for those 50 entries cannot be fully excluded, and authors planning to cite from this corpus for downstream work should perform direct DOI+author cross-verification of any specific entry they rely upon.

### 9.3 Responsible Use Statement

GNSS interference mitigation research is dual-use by nature: techniques that defend civilian PNT against jamming share mathematical and engineering foundations with techniques that could be used to characterise or design interference sources. This review is scoped to *receiver-side* mitigation, taxonomy, and failure-mode analysis with the explicit purpose of strengthening civilian PNT resilience in aviation, maritime, telecommunications, finance, and emergency-services contexts. Three principles guide the responsible-use posture of this work: (1) no operational jammer construction details, transmitter design parameters, or detection-evasion guidance are provided, beyond the threat taxonomy required for mitigation design; (2) jammer-source geolocation, where mentioned (Murrian et al., 2021), is reported at the published level of specificity from operationally relevant academic sources and does not extend beyond their disclosure; (3) the cited literature is intended for defensive engineering, regulatory standards work (e.g., RTCA, EUROCAE, EU Commission JRC programmes), and academic research, not for operational targeting. Practitioners adapting findings for fielded systems should consult applicable national radio-spectrum regulations and aviation/maritime safety authorities. Reproduction of the source corpus and synthesis findings for educational and engineering purposes is appropriate; adaptation for offensive interference design is outside the intended use.

---

## References

Amin, M. G., Borio, D., Zhang, Y. D., & Galleani, L. (2017). Time-frequency analysis for GNSSs: From interference mitigation to system monitoring. *IEEE Signal Processing Magazine*, *34*(5), 85–95. https://doi.org/10.1109/MSP.2017.2710235

Amin, M. G., Wang, X., Zhang, Y. D., Ahmad, F., & Aboutanios, E. (2016). Sparse arrays and sampling for interference mitigation and DOA estimation in GNSS. *Proceedings of the IEEE*, *104*(6), 1302–1317. https://doi.org/10.1109/JPROC.2016.2525926

Bastide, F., Chatre, E., Macabiau, C., & Roturier, B. (2004, January). *GPS L5 and Galileo E5a/E5b signal-to-noise density ratio degradation due to DME/TACAN signals: Simulations and theoretical derivation*. Proceedings of the ION NTM 2004, San Diego, CA (pp. 1049–1062).

Borio, D. (2016). Loop analysis of adaptive notch filters. *IET Signal Processing*, *10*(6), 659–669. https://doi.org/10.1049/iet-spr.2015.0310

Borio, D. (2016). Swept GNSS jamming mitigation through pulse blanking. In *Proceedings of the 2016 European Navigation Conference (ENC)*, Helsinki, Finland. https://doi.org/10.1109/EURONAV.2016.7530549

Borio, D., & Closas, P. (2017). A fresh look at GNSS anti-jamming. *Inside GNSS*, *12*(6), 54–61.

Borio, D., & Closas, P. (2019). Robust transform domain signal processing for GNSS. *NAVIGATION: Journal of the Institute of Navigation*, *66*(2), 305–323. https://doi.org/10.1002/navi.300

Borio, D., Dovis, F., Kuusniemi, H., & Lo Presti, L. (2016). Impact and detection of GNSS jammers on consumer grade satellite navigation receivers. *Proceedings of the IEEE*, *104*(6), 1233–1245. https://doi.org/10.1109/JPROC.2016.2543266

Borio, D., & Gioia, C. (2021). GNSS interference mitigation: A measurement and position domain assessment. *NAVIGATION: Journal of the Institute of Navigation*, *68*(1), 93–114. https://doi.org/10.1002/navi.391

Borio, D., & Gioia, C. (2021). Interference mitigation: Impact on GNSS timing. *GPS Solutions*, *25*(2), Article 65. https://doi.org/10.1007/s10291-020-01075-x

Brennan, D. G. (1959). Linear diversity combining techniques. *Proceedings of the IRE*, *47*(6), 1075–1102. https://doi.org/10.1109/JRPROC.1959.287136

Broumandan, A., Jafarnia-Jahromi, A., Daneshmand, S., & Lachapelle, G. (2016). Overview of spatial processing approaches for GNSS structural interference detection and mitigation. *Proceedings of the IEEE*, *104*(6), 1246–1257. https://doi.org/10.1109/JPROC.2016.2529600

Cortés, I., van der Merwe, J. R., Lohan, E. S., Nurmi, J., & Felber, W. (2023). Evaluation of low-complexity adaptive full direct-state Kalman filter for robust GNSS tracking. *Sensors*, *23*(7), Article 3658. https://doi.org/10.3390/s23073658

Cortés, I., van der Merwe, J. R., Nurmi, J., Rügamer, A., & Felber, W. (2021). Evaluation of adaptive loop-bandwidth tracking techniques in GNSS receivers. *Sensors*, *21*(2), Article 502. https://doi.org/10.3390/s21020502

Cuntz, M., Konovaltsev, A., & Meurer, M. (2016). Concepts, development, and validation of multiantenna GNSS receivers for resilient navigation. *Proceedings of the IEEE*, *104*(6), 1288–1301. https://doi.org/10.1109/JPROC.2016.2535500

Daneshmand, S., Marathe, T., & Lachapelle, G. (2016). Millimetre level accuracy GNSS positioning with the blind adaptive beamforming method in interference environments. *Sensors*, *16*(11), Article 1824. https://doi.org/10.3390/s16111824

Dovis, F. (Ed.). (2015). *GNSS interference, threats, and countermeasures*. Artech House. ISBN 978-1-60807-810-3.

Fante, R. L., & Vaccaro, J. J. (2000). Wideband cancellation of interference in a GPS receive array. *IEEE Transactions on Aerospace and Electronic Systems*, *36*(2), 549–564. https://doi.org/10.1109/7.845241

Felux, M., Fol, P., Figuet, B., Waltert, M., & Olive, X. (2024). Impacts of global navigation satellite system jamming on aviation. *NAVIGATION: Journal of the Institute of Navigation*, *71*(3), Article navi.657. https://doi.org/10.33012/navi.657

Fernández-Prades, C., Arribas, J., & Closas, P. (2016). Robust GNSS receivers by array signal processing: Theory and implementation. *Proceedings of the IEEE*, *104*(6), 1207–1220. https://doi.org/10.1109/JPROC.2016.2532963

Gao, G. X., Heng, L., Hornbostel, A., Denks, H., Meurer, M., Walter, T., & Enge, P. (2013). DME/TACAN interference mitigation for GNSS: Algorithms and flight test results. *GPS Solutions*, *17*(4), 561–573. https://doi.org/10.1007/s10291-012-0301-9

Gao, G. X., Sgammini, M., Lu, M., & Kubo, N. (2016). Protecting GNSS receivers from jamming and interference. *Proceedings of the IEEE*, *104*(6), 1327–1338. https://doi.org/10.1109/JPROC.2016.2525938

Garcia-Peña, A., Julien, O., Macabiau, C., Mabilleau, M., & Durel, P. (2021). GNSS C/N0 degradation model in presence of continuous wave and pulsed interference. *NAVIGATION: Journal of the Institute of Navigation*, *68*(1), 75–91. https://doi.org/10.1002/navi.405

Gioia, C., & Borio, D. (2023). Multi-layered multi-constellation global navigation satellite system interference mitigation. *NAVIGATION: Journal of the Institute of Navigation*, *70*(4), Article navi.596. https://doi.org/10.33012/navi.596

Groves, P. D. (2013). *Principles of GNSS, inertial, and multisensor integrated navigation systems* (2nd ed.). Artech House. ISBN 978-1-60807-005-3.

Ioannides, R. T., Pany, T., & Gibbons, G. (2016). Known vulnerabilities of global navigation satellite systems, status, and potential mitigation techniques. *Proceedings of the IEEE*, *104*(6), 1174–1194. https://doi.org/10.1109/JPROC.2016.2535898

Jafarnia-Jahromi, A., Broumandan, A., Daneshmand, S., & Lachapelle, G. (2015, September). *Vulnerability analysis of civilian L1/E1 GNSS signals against different types of interference*. Proceedings of the ION GNSS+ 2015, Tampa, FL.

Kaplan, E. D., & Hegarty, C. J. (Eds.). (2017). *Understanding GPS/GNSS: Principles and applications* (3rd ed.). Artech House. ISBN 978-1-63081-058-0.

Kim, S., Gault, N., Jo, Y., Yoon, H., Park, B., Garcia-Peña, A., Macabiau, C., & Akos, D. M. (2025). GNSS L5/E5a code properties in the presence of a blanker. *NAVIGATION: Journal of the Institute of Navigation*, *72*(2), Article navi.700. https://doi.org/10.33012/navi.700

Lashley, M., Bevly, D. M., & Hung, J. Y. (2010). Analysis of deeply integrated and tightly coupled architectures. In *Proceedings of the 2010 IEEE/ION Position Location and Navigation Symposium (PLANS)*, 382–396. https://doi.org/10.1109/PLANS.2010.5507282

Lashley, M., & Bevly, D. M. (2015). Performance of a MEMS IMU deeply coupled with a GNSS receiver under jamming. In *Proceedings of the 2015 IEEE/ION Position, Location and Navigation Symposium (PLANS)*. https://doi.org/10.1109/PLANS.2015.7033711

Mitch, R. H., Dougherty, R. C., Psiaki, M. L., Powell, S. P., O'Hanlon, B. W., Bhatti, J. A., & Humphreys, T. E. (2011, September). *Signal characteristics of civil GPS jammers*. Proceedings of the ION GNSS 2011, Portland, OR (pp. 1907–1919).

Mitch, R. H., Psiaki, M. L., & Ertan, T. (2016). Chirp-style GNSS jamming signal tracking and geolocation. *NAVIGATION: Journal of the Institute of Navigation*, *63*(1), 15–37. https://doi.org/10.1002/navi.128

Morales-Ferre, R., de la Fuente, A., & Lohan, E. S. (2019). Jammer classification in GNSS bands via machine learning algorithms. *Sensors*, *19*(22), Article 4841. https://doi.org/10.3390/s19224841

Morales-Ferre, R., Richter, P., Falletti, E., de la Fuente, A., & Lohan, E. S. (2020). A survey on coping with intentional interference in satellite navigation for manned and unmanned aircraft. *IEEE Communications Surveys & Tutorials*, *22*(1), 249–291. https://doi.org/10.1109/COMST.2019.2949178

Morong, T., Puričer, P., & Kovář, P. (2019). Study of the GNSS jamming in real environment. *International Journal of Electronics and Telecommunication*, *65*(1), 65–70. https://doi.org/10.24425/ijet.2019.126284

Mosavi, M. R., & Shafiee, F. (2016). Narrowband interference suppression for GPS navigation using neural networks. *GPS Solutions*, *20*(2), 341–351. https://doi.org/10.1007/s10291-015-0442-8

Mosavi, M. R., Rezaei, M. J., Pashaian, M., & Moghaddasi, M. N. (2017). A fast and accurate anti-jamming system based on wavelet packet transform for GPS receivers. *GPS Solutions*, *21*(1), 415–426. https://doi.org/10.1007/s10291-016-0535-z

Murrian, M. J., Narula, L., Iannucci, P. A., Budzien, S., O'Hanlon, B. W., & Humphreys, T. E. (2021). First results from three years of GNSS interference monitoring from low Earth orbit. *NAVIGATION: Journal of the Institute of Navigation*, *68*(4), 673–685. https://doi.org/10.1002/navi.449

Musumeci, L., & Dovis, F. (2014). Use of the wavelet transform for interference detection and mitigation in global navigation satellite systems. *International Journal of Navigation and Observation*, *2014*, Article 262186. https://doi.org/10.1155/2014/262186

Osechas, O., Fohlmeister, F., Dautermann, T., & Felux, M. (2022). Impact of GNSS-band radio interference on operational avionics. *NAVIGATION: Journal of the Institute of Navigation*, *69*(2), Article navi.516. https://doi.org/10.33012/navi.516

Pullen, S., & Gao, G. X. (2012). GNSS jamming in the name of privacy: Potential threat to GPS aviation. *Inside GNSS*, *7*(2), 34–43.

Radoš, K., Brkić, M., & Begušić, D. (2024). Recent advances on jamming and spoofing detection in GNSS. *Sensors*, *24*(13), Article 4210. https://doi.org/10.3390/s24134210

Ramezanpour, P., & Mosavi, M. R. (2020). DNN-based interference mitigation beamformer. *IET Radar, Sonar & Navigation*, *14*(11), 1788–1794. https://doi.org/10.1049/iet-rsn.2020.0234

Ramezanpour, P., Rezaei, M. J., & Mosavi, M. R. (2020). Deep-learning-based beamforming for rejecting interferences. *IET Signal Processing*, *14*(8), 467–473. https://doi.org/10.1049/iet-spr.2019.0495

Swinney, C. J., & Woods, J. C. (2021). GNSS jamming classification via CNN, transfer learning & the novel concatenation of signal representations. In *Proceedings of the 2021 International Conference on Cyber Situational Awareness, Data Analytics and Assessment (CyberSA)*. https://doi.org/10.1109/CyberSA52016.2021.9478250

Zhang, Y. D., Amin, M. G., & Wang, B. (2016). Mitigation of sparsely sampled nonstationary jammers for multi-antenna GNSS receivers. In *Proceedings of the 2016 IEEE International Conference on Acoustics, Speech and Signal Processing (ICASSP)*, 6565–6569. https://doi.org/10.1109/ICASSP.2016.7472942
