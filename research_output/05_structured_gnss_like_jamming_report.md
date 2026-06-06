# Structured GNSS-Like Selective DoS Jamming: Research Framing, Detection, Dataset, and Mitigation Plan

**Date:** 2026-05-12  
**Repository context:** `gps-sdr-sim` selective attack and synthetic-satellite extensions  
**Status:** working research note for paper/dataset planning

---

## 1. Executive Summary

The final experiment should be framed as **GNSS-like selective denial-of-service (DoS) jamming** or **structured signal-like jamming**.

The term "jamming" is defensible because the operational objective and outcome are denial of PVT, not false-position control. However, the waveform is not conventional noise, chirp, CW, or broadband RFI. It is GPS-like signal injection over a selected subset of satellites. The cleanest phrasing for a paper is:

> This work studies a spoofing-like waveform with a jamming objective: selected GPS-like satellite signals are injected alongside live-sky signals so that the receiver can acquire and track channels, but cannot form a consistent PVT solution.

This distinction matters because most anti-jamming work detects or mitigates abnormal RF energy before despreading, while this scenario can pass RF, ADC/pre-correlation, correlator, and tracking stages. The failure is primarily exposed at the **navigation consistency layer**.

The likely contribution is publishable if packaged as:

1. A labelled COTS-receiver dataset showing PVT denial under mixed live-sky and GPS-like injected satellite signals.
2. A receiver-side consistency monitor using raw observables to detect mixed-basis measurement inconsistency.
3. A mitigation baseline that recovers PVT only when enough authentic redundancy remains, and otherwise raises a reliable DoS alarm.

---

## 2. Threat Model

### 2.1 Attack Class

The experiment is not classic brute-force jamming. It is also not classic spoofing if the attacker does not try to force a coherent false PVT solution. It is a hybrid class:

| Property | Classic jamming | Classic spoofing | This experiment |
|---|---|---|---|
| Main objective | Deny receiver operation | Control false PVT | Deny PVT |
| Waveform | Noise, chirp, CW, pulsed | GNSS-like | GNSS-like |
| Navigation message | Usually absent | Usually coherent false nav/PVT | Correct or plausible nav message |
| Receiver tracking | Often lost | Often maintained | Maintained for injected channels |
| Failure stage | RF/tracking | Navigation/authentication | Navigation consistency |

Recommended term:

> **Structured GNSS-like selective DoS jamming**

Alternative terms:

- GNSS-like PVT-denial jamming
- partial-constellation signal-like jamming
- selective navigation-layer DoS
- spoofing-like jamming

The paper should avoid calling it only "spoofing" because the intended effect is not a controlled false solution. It should also avoid calling it only "jamming" without qualification because reviewers may expect RF-noise denial.

### 2.2 Relation to Existing Work

The closest threat-model paper is Caparra et al. (2018), which describes low-power selective DoS attacks that target individual ranging signals and disrupt PNT by attacking the receiver correlation process rather than overpowering the whole band. That work is a strong anchor for the "selective DoS jamming" terminology.

Your experiment differs in emphasis:

- Caparra et al. focus on disrupting lock indicators and observables at the correlation/tracking level.
- This experiment shows a later-stage denial mode: the receiver can still track GPS-like injected channels, but the mixed measurement set cannot support a valid PVT solution.

This is a useful gap: **tracking success does not imply navigation consistency**.

---

## 3. Receiver-Chain Interpretation

For a single-antenna receiver such as a u-blox ZED-F9P, the attack is expected to pass the early receiver chain.

| Receiver stage | Expected behavior under structured GNSS-like selective DoS |
|---|---|
| RF / antenna | Likely passes. A single patch antenna cannot distinguish satellite-origin signals from same-band local GNSS-like signals by direction. |
| ADC / pre-correlation | Likely passes. No necessary broadband, chirp, CW, or abnormal power signature. |
| Correlator / acquisition | Can pass. Injected signals are GNSS-like enough to acquire. |
| Tracking | Can pass. Code/carrier tracking may remain stable for injected channels. |
| Navigation / PVT | Main failure point. The full measurement set cannot be fit by one receiver position and clock state, or PVT is denied by the receiver's internal consistency checks. |

This receiver-chain behavior is the core research message:

> The attack does not need to break acquisition or tracking. It can preserve tracking while denying PVT by creating a measurement set that is internally inconsistent at the navigation layer.

This is different from PRN-selective matched-code noise, where the target channels degrade at the correlator/tracking stage. Here the suspicious evidence may only appear after raw measurements are combined into a PVT solution.

---

## 4. Is This "Jamming"?

Yes, if the paper defines the term by **effect and intent** rather than by waveform family.

A practical definition:

> Jamming is an intentional interference condition whose objective is denial or degradation of PNT service.

Under that definition, the experiment is jamming because the receiver is denied PVT. The waveform is GNSS-like, so the report should use a qualified term: **GNSS-like jamming** or **structured signal-like jamming**.

The responsible taxonomy is:

- **Conventional jamming:** denial via raised interference power or spectral occupancy.
- **Selective correlation-domain jamming:** denial via PRN-specific interference or lock-indicator degradation.
- **Structured GNSS-like navigation-domain jamming:** denial via GPS-like signals that remain trackable but make the navigation solution inconsistent.

Your experiment falls in the third category.

---

## 5. Residual Checks as Detection

### 5.1 Why Residual Checks Are Natural Here

For GPS single-point positioning, the receiver estimates four unknowns:

```text
x, y, z, receiver_clock_bias
```

For each satellite measurement:

```text
pseudorange_i = geometric_range(receiver, satellite_i) + receiver_clock_bias + error_i
```

If there are more than four measurements, the receiver can test whether all measurements agree with one state. If live-sky and injected measurements belong to incompatible time/range bases, no single receiver position and clock bias can fit them all. The residual vector after least-squares positioning becomes large.

A standard weighted residual statistic is:

```text
T = r^T W r
```

where `r` is the post-fit pseudorange residual vector and `W` is a weighting matrix, often based on measurement variance or C/N0. Classical RAIM compares this statistic against a threshold, often using a chi-square model with degrees of freedom related to the number of redundant measurements.

So for this attack, residual checks are not just reasonable; they are the first baseline a reviewer will expect.

### 5.2 When Residual Checks Are Easy

Residual checks are easy when:

- raw pseudorange measurements are available, such as via `UBX-RXM-RAWX`;
- there are more than four usable measurements;
- the attack is partial, so authentic and injected measurements conflict;
- the authentic subset still has enough satellites and acceptable geometry;
- the monitor can compute or reproduce satellite positions from the ephemeris.

In this case, the mitigation can be:

1. solve PVT with all measurements;
2. compute residual statistic;
3. if inconsistent, search for a consistent subset;
4. output PVT from the best subset if it has enough satellites;
5. otherwise raise a structured-DoS alarm.

### 5.3 When Residual Checks Are Not Enough

Residual checks are not a complete solution:

- If only four measurements are used, there is no redundancy for a residual test.
- If the attacker supplies a full self-consistent synthetic constellation, residuals can be low.
- If many measurements are corrupted together, single-fault RAIM/FDE assumptions break.
- If the authentic subset is too small or geometrically weak, detection may be possible but PVT recovery is not.
- If the COTS receiver hides measurements before the monitor sees them, external mitigation becomes harder.

Therefore the honest mitigation claim should be:

> The proposed monitor detects structured GNSS-like DoS and can recover PVT when sufficient authentic redundancy remains. If redundancy is insufficient, it provides fail-safe denial detection rather than guaranteed PVT recovery.

That claim is technically defensible.

---

## 6. Existing Literature on Residual and Consistency Checks

Residual and consistency checks are established in anti-spoofing and integrity-monitoring literature.

### 6.1 RAIM and Collaborative RAIM

Wang et al. (2016) explicitly state that conventional RAIM can detect spoofing when authentic and spoofed measurements are used together because the measurements are inconsistent. They also note the key limitation: if all receiver channels are misled coherently, conventional RAIM can fail. Their Collaborative RAIM adds inter-receiver distance constraints, which is outside the single-receiver scope of this project but useful as evidence that residual inconsistency is a known detection mechanism.

Relevance to this project:

- supports residual checks as a baseline;
- confirms the partial/mixed-signal case is detectable by consistency;
- highlights that full coherent attacks defeat simple residual methods.

### 6.2 Sliding-Window Pseudorange Detection

Jeong (2020) proposes a maximum-likelihood sliding-window method using receiver pseudorange measurements. The paper also reviews conventional RAIM residual checks and chi-square testing. This is relevant because the proposed detector uses low-cost receiver outputs rather than special antenna hardware.

Relevance to this project:

- supports a receiver-output-only detection method;
- provides a time-series alternative to single-epoch residual checks;
- helps address false alarms from instantaneous measurement noise.

### 6.3 Pseudorange-Doppler Consistency

Chu et al. (2019) propose checking consistency between Doppler-based and pseudorange-based positioning results, with GLRT-based detection and validation on software-defined and real-time receiver platforms.

Relevance to this project:

- structured GNSS-like injection may preserve tracking but disturb pseudorange/Doppler consistency;
- Doppler consistency can strengthen a pseudorange-only residual detector;
- this is a useful second baseline after RAIM residuals.

### 6.4 Clock-State Self-Consistency

Chen et al. (2026) propose self-consistent verification of receiver clock state by comparing estimated clock bias with clock bias integrated from clock drift. Their method is designed for spoofing detection and reports applicability to partial-channel and full-channel spoofing.

Relevance to this project:

- very relevant to mixed time-basis inconsistency;
- single-receiver and no auxiliary hardware;
- useful as a SOTA baseline for navigation-layer anomaly detection.

### 6.5 General Spoofing-Defense Taxonomy

Psiaki and Humphreys (2016) provide a broad taxonomy of GNSS spoofing attacks and defenses. It is useful for positioning this work between spoofing and jamming.

Relevance to this project:

- supports the hybrid terminology;
- gives reviewers a familiar map of detection stages;
- helps separate RF/correlation defenses from navigation-consistency defenses.

---

## 7. Does Current SOTA Detect This Attack?

The answer is mixed.

### Likely to Detect

Navigation-layer monitors can detect this attack if they have the right observables:

- RAIM/FDE residual checks;
- robust subset selection over pseudorange measurements;
- pseudorange-Doppler consistency checks;
- receiver clock-bias/clock-drift consistency checks;
- known-position checks for static timing receivers;
- multi-antenna angle-of-arrival checks, if hardware is available.

For this project, the single-receiver methods are the relevant ones:

- RAIM/FDE residual checks;
- sliding-window pseudorange monitoring;
- pseudorange-Doppler consistency;
- clock-state self-consistency.

### Likely Not to Detect Reliably

These methods are weak for this attack:

- AGC or RF power alarms;
- spectrum-only detectors;
- notch filters;
- pulse blanking;
- RIM/ZMNL methods aimed at non-GNSS-like interference;
- C/N0-only monitoring;
- basic correlation-shape signal-quality monitoring, if the injected signals form clean single peaks;
- ML classifiers trained only on CW/chirp/noise jammer spectrograms.

The important point:

> This attack is not primarily an RF anomaly. It is a measurement-set consistency anomaly.

### Not Fully Solved

Even if residual checks detect the condition, mitigation is not always possible. Detection and recovery are different claims.

| Situation | Detection | PVT recovery |
|---|---|---|
| Mixed authentic + injected, enough authentic satellites remain | Likely | Possible |
| Mixed authentic + injected, authentic subset too small | Likely | No |
| Full coherent synthetic constellation | Maybe not by residual only | Receiver may solve false PVT |
| Only four measurements | Weak | Weak |
| No raw observables exposed | Receiver-dependent | Hard externally |

So the paper should not claim "we solve structured GNSS-like jamming." The stronger and safer claim is:

> We provide a dataset and a baseline detector/mitigator for structured GNSS-like selective DoS. The method recovers PVT when authentic redundancy remains and raises an alarm otherwise.

---

## 8. Dataset Contribution

The dataset can be a main contribution if it is documented and released cleanly.

### 8.1 Dataset Claim

Suggested contribution statement:

> We release a labelled COTS-receiver dataset of structured GNSS-like selective DoS attacks, where a u-blox ZED-F9P tracks both live-sky and injected GPS-like satellite channels while PVT is denied by measurement-set inconsistency.

This is valuable because many public GNSS interference datasets focus on RF jamming, spoofing takeovers, simulated interference, or clean receiver logs. A dataset showing **trackable but PVT-denying partial GNSS-like injection** fills a narrower and useful gap.

### 8.2 Required Dataset Artifacts

Each scenario should include:

- receiver model and firmware version;
- receiver configuration export;
- antenna model and placement description;
- location and approximate test environment;
- RINEX navigation file used by the generator;
- generated scenario configuration, including injected PRNs and labels;
- raw receiver logs:
  - `UBX-RXM-RAWX`;
  - `UBX-NAV-PVT`;
  - `UBX-NAV-SAT`;
  - `UBX-NAV-SIG`;
  - `UBX-NAV-STATUS`;
  - `UBX-MON-RF`;
- attack phase labels:
  - clean baseline;
  - injection active;
  - post-injection recovery;
- per-PRN label:
  - live sky;
  - injected;
  - ambiguous/unknown;
- outcome label:
  - PVT valid;
  - PVT denied;
  - PVT degraded;
  - receiver spoof flag state;
  - receiver jamming indicator state if available.

### 8.3 Minimum Experimental Matrix

A useful first dataset does not need many scenarios. It needs clean labels and repeated runs.

Recommended matrix:

| Scenario | Purpose |
|---|---|
| Clean live-sky baseline | Measure normal residuals and receiver state |
| Injection only, non-conflicting weak case | Verify receiver can track injected signals |
| Structured DoS, 1 injected PRN | Test low severity |
| Structured DoS, 2-3 injected PRNs | Test transition to PVT degradation |
| Structured DoS at edge of PVT loss | Main contribution |
| Recovery after injection stops | Measure recovery time |
| Repeated runs across different satellite geometries | Show geometry dependence |

The key measurement is not only whether PVT is denied, but **which stage still looks healthy**:

- C/N0 normal or plausible;
- tracking maintained;
- RF indicators normal or weakly affected;
- residuals inconsistent;
- PVT denied or receiver excludes one group.

---

## 9. Mitigation Baseline

### 9.1 Proposed Single-Receiver Monitor

A practical monitor can be implemented outside the receiver using raw observables:

1. Parse raw pseudorange, Doppler, C/N0, satellite ID, and time tags.
2. Compute satellite positions from the broadcast ephemeris.
3. Run weighted least-squares PVT over all candidate measurements.
4. Compute weighted residual statistic.
5. If residual is normal, accept the epoch.
6. If residual is abnormal, run robust subset selection.
7. If a consistent subset exists with enough satellites and acceptable DOP, output recovered PVT and alarm.
8. If no consistent subset exists, output structured-DoS alarm and mark PVT unavailable.

The subset search can start simple:

- leave-one-out FDE;
- leave-k-out for small k;
- RANSAC/MLESAC-style subset search;
- largest-consistent-subset selection.

### 9.2 Add Doppler and Clock-State Consistency

Residual checks should be the baseline, not the final system. The stronger system combines:

- pseudorange residual RAIM;
- Doppler-position vs pseudorange-position consistency;
- clock bias vs integrated clock drift consistency;
- per-PRN time-series monitoring.

This combination is well matched to the attack because the injected signals may be trackable, but the full measurement set still violates physical receiver-clock and geometry consistency.

### 9.3 Expected Outcomes

The mitigation should report three possible states:

| State | Meaning |
|---|---|
| `clean` | All measurements fit one receiver state |
| `recoverable_structured_dos` | Inconsistency detected; authentic subset still supports PVT |
| `unrecoverable_structured_dos` | Inconsistency detected; no reliable PVT can be recovered |

This avoids overstating the method. A fail-safe alarm is a valid mitigation when PVT recovery is impossible.

---

## 10. Evaluation Plan

### 10.1 Research Questions

Suggested paper research questions:

1. Can a COTS GNSS receiver track structured GPS-like injected signals while failing to solve PVT?
2. Which receiver-chain indicators remain normal under this attack?
3. Can single-receiver residual and consistency monitors detect the attack from raw observables?
4. Under what satellite redundancy and geometry conditions can PVT be recovered?

### 10.2 Metrics

Detection metrics:

- detection rate;
- false alarm rate on clean live-sky data;
- time to alarm;
- alarm lead time before PVT loss, if any;
- attack-stage classification accuracy.

Mitigation metrics:

- PVT recovery rate;
- recovered position error;
- number of satellites retained/excluded;
- DOP of recovered subset;
- time to recover after attack ends.

Receiver-chain metrics:

- C/N0 per PRN;
- lock/quality indicators per PRN;
- raw pseudorange residuals;
- Doppler residuals;
- receiver clock bias and drift estimates;
- u-blox spoofing and jamming indicators;
- PVT fix type and validity.

### 10.3 Baselines

Recommended baselines:

- C/N0 thresholding;
- receiver spoof flag only;
- receiver jamming indicator only;
- single-epoch RAIM residual check;
- RAIM + FDE;
- sliding-window pseudorange detector;
- pseudorange-Doppler consistency detector;
- clock-state self-consistency detector.

The paper becomes stronger if it shows that RF/C/N0 detectors miss or lag the attack, while navigation-consistency detectors catch it.

---

## 11. Main Paper Claim

A strong but defensible claim:

> We demonstrate a structured GNSS-like selective DoS attack against a COTS receiver in which injected GPS-like satellite signals remain trackable and carry correct navigation data, yet deny PVT by making the measurement set inconsistent. We release labelled receiver logs and provide a single-receiver consistency-monitor baseline that detects the attack and recovers PVT when sufficient authentic redundancy remains.

Avoid claiming:

- "first ever" unless the literature check is expanded and confirmed;
- "solves GNSS jamming";
- "undetectable";
- "bypasses all anti-spoofing";
- "works against all receivers."

Better novelty language:

- "under-characterized";
- "not covered by RF-centric jamming datasets";
- "bridges selective jamming and partial spoofing";
- "demonstrates tracking-preserving PVT denial on a COTS receiver";
- "provides a labelled dataset for navigation-layer DoS detection."

---

## 12. Limitations and Reviewer Risks

### 12.1 Taxonomy Risk

Reviewers may argue this is spoofing, not jamming. The answer is to state clearly:

- waveform: spoofing-like;
- objective: jamming/DoS;
- failure mode: navigation-layer PVT denial.

### 12.2 Single-Receiver Scope

The work intentionally scopes to one receiver. This excludes collaborative RAIM, array antennas, and networked defenses. That is acceptable if stated as a design constraint.

### 12.3 Receiver-Specific Behavior

u-blox ZED-F9P internal checks are proprietary. The dataset shows observed receiver behavior, but not the internal reason for PVT denial. The mitigation should therefore be implemented as an external raw-observable monitor, not claimed as the same method used internally by u-blox.

### 12.4 Recovery Is Geometry-Dependent

If the attack removes too much authentic redundancy, no single-receiver algorithm can recover PVT. The correct behavior is alarm and fail-safe unavailability.

### 12.5 Responsible-Use Boundary

The public paper/dataset should avoid transmitter construction details, power settings, or operational guidance. It should release receiver observables and scenario labels sufficient for defensive detector research.

---

## 13. Recommended Next Steps

1. Produce fresh logs for the final scenario with the old issue note treated as non-authoritative.
2. Ensure every run includes `RAWX`, `NAV-PVT`, `NAV-SAT`, `NAV-SIG`, `NAV-STATUS`, and `MON-RF`.
3. Build a small parser that exports per-epoch CSV:
   - pseudorange;
   - Doppler;
   - C/N0;
   - satellite used/not used;
   - fix status;
   - spoof/jam indicators;
   - injected/live labels.
4. Implement the first RAIM residual baseline.
5. Add robust subset selection.
6. Add Doppler and clock-state consistency baselines.
7. Write the paper around the dataset and mitigation baseline, not only around the attack.

---

## References

Caparra, G., Ceccato, S., Formaggio, F., Laurenti, N., & Tomasin, S. (2018). **Low Power Selective Denial of Service Attacks Against GNSS.** *ION GNSS+ 2018*, 3028-3041. https://doi.org/10.33012/2018.15909  
Accessible abstract/PDF page: https://www.ion.org/publications/pdf.cfm?articleID=15909

Wang, F., Li, H., Yang, Y., & Lu, M. (2016). **GNSS Spoofing Detection Based on Collaborative RAIM.** *ION ITM 2016*, 748-755. https://doi.org/10.33012/2016.13458  
ION page: https://www.ion.org/publications/abstract.cfm?articleID=13458

Jeong, S. (2020). **GNSS spoofing detection using a maximum likelihood-based sliding window method.** *PLOS ONE*, 15(8), e0237146. https://doi.org/10.1371/journal.pone.0237146  
Open access: https://pmc.ncbi.nlm.nih.gov/articles/PMC7454987/

Chu, F., Li, H., Wen, J., & Lu, M. (2019). **Statistical Model and Performance Evaluation of a GNSS Spoofing Detection Method Based on the Consistency of Doppler and Pseudorange Positioning Results.** *The Journal of Navigation*, 72(2), 447-466. https://doi.org/10.1017/S0373463318000747  
Cambridge page: https://www.cambridge.org/core/journals/journal-of-navigation/article/abs/statistical-model-and-performance-evaluation-of-a-gnss-spoofing-detection-method-based-on-the-consistency-of-doppler-and-pseudorange-positioning-results/E716A3A4127AA590DD8A54EB944FAABA

Chen, Y., Jiang, Y., Wen, C., Liu, Y., Wang, L., He, X., Jiang, Y., Peng, X., Liu, X., et al. (2026). **GNSS Spoofing Detection via Self-Consistent Verification of Receiver's Clock State.** *Sensors*, 26(2), 397. https://doi.org/10.3390/s26020397  
Open access: https://www.mdpi.com/1424-8220/26/2/397

Psiaki, M. L., & Humphreys, T. E. (2016). **GNSS Spoofing and Detection.** *Proceedings of the IEEE*, 104(6), 1258-1270. https://doi.org/10.1109/JPROC.2016.2526658  
Metadata page: https://dblp.org/rec/journals/pieee/PsiakiH16

Curran, J. T. (Inside GNSS). **Systemic Jamming.** https://insidegnss.com/systemic-jamming/

